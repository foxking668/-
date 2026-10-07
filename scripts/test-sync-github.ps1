# Integration tests use disposable local repositories, never the real GitHub origin.
$ErrorActionPreference = 'Stop'
$testRoot = Join-Path ([IO.Path]::GetTempPath()) ('loongson-sync-test-' + [Guid]::NewGuid().ToString('N'))
$remotePath = Join-Path $testRoot 'remote.git'
$checkoutPath = Join-Path $testRoot 'checkout'
$otherCheckout = Join-Path $testRoot 'other'
$scriptSource = Join-Path $PSScriptRoot 'sync-github.ps1'
$powershellExecutable = (Get-Command powershell -ErrorAction Stop).Source
$checks = 0

function Invoke-TestGit {
    param([string[]]$GitArguments)
    $ErrorActionPreference = 'Continue'
    $output = @(& git @GitArguments 2>&1)
    if ($LASTEXITCODE -ne 0) { throw "Test setup git failed: $($output -join "`n")" }
    $output | ForEach-Object { $_.ToString() }
}

function Assert-Equal {
    param($Actual, $Expected, [string]$Description)
    if ($Actual -cne $Expected) { throw "FAIL: $Description (actual=$Actual expected=$Expected)" }
    $script:checks++
}

function Invoke-TestSync {
    param([switch]$DryRun, [switch]$ShouldFail)
    $scriptPath = Join-Path $checkoutPath 'scripts/sync-github.ps1'
    $arguments = @('-NoProfile', '-ExecutionPolicy', 'Bypass', '-File', $scriptPath,
        '-ExpectedRemote', $remotePath, '-Message', 'Review source change; literal $(do-not-execute)', '-BranchSlug', 'reviewed-source')
    if ($DryRun) { $arguments += '-DryRun' }
    $ErrorActionPreference = 'Continue'
    $output = @(& $powershellExecutable @arguments 2>&1)
    $exitCode = $LASTEXITCODE
    if ($ShouldFail) {
        if ($exitCode -eq 0) { throw 'A failing sync unexpectedly succeeded.' }
        if (($output -join "`n") -match 'SUCCESS:') { throw 'A failing sync reported success.' }
    } elseif ($exitCode -ne 0) { throw "Sync failed unexpectedly:`n$($output -join "`n")" }
    $script:checks++
    $output | ForEach-Object { $_.ToString() }
}

New-Item -ItemType Directory -Path $testRoot | Out-Null
try {
    Invoke-TestGit @('init', '--bare', '--initial-branch=main', $remotePath) | Out-Null
    Invoke-TestGit @('init', '--initial-branch=main', $checkoutPath) | Out-Null
    Invoke-TestGit @('-C', $checkoutPath, 'config', 'user.name', 'Sync integration test') | Out-Null
    Invoke-TestGit @('-C', $checkoutPath, 'config', 'user.email', 'sync-test@example.invalid') | Out-Null
    New-Item -ItemType Directory -Path (Join-Path $checkoutPath 'SMART_CAR_2026'), (Join-Path $checkoutPath 'scripts') | Out-Null
    Copy-Item -LiteralPath $scriptSource -Destination (Join-Path $checkoutPath 'scripts/sync-github.ps1')
    $sourcePath = Join-Path $checkoutPath 'SMART_CAR_2026/source.txt'
    $optionalDocument = Join-Path $checkoutPath 'GITHUB_SYNC.md'
    [IO.File]::WriteAllText($sourcePath, 'initial')
    [IO.File]::WriteAllText($optionalDocument, 'tracked document to delete')
    Invoke-TestGit @('-C', $checkoutPath, 'add', '.') | Out-Null
    Invoke-TestGit @('-C', $checkoutPath, 'commit', '-m', 'initial') | Out-Null
    Invoke-TestGit @('-C', $checkoutPath, 'remote', 'add', 'origin', $remotePath) | Out-Null
    Invoke-TestGit @('-C', $checkoutPath, 'push', '-u', 'origin', 'main') | Out-Null
    $initialCommit = Invoke-TestGit @('-C', $checkoutPath, 'rev-parse', 'HEAD')

    [IO.File]::WriteAllText($sourcePath, 'first reviewed edit')
    Remove-Item -LiteralPath $optionalDocument
    [IO.File]::WriteAllText((Join-Path $checkoutPath 'outside.txt'), 'do not upload')
    Invoke-TestSync -DryRun | Out-Null
    Assert-Equal (Invoke-TestGit @('-C', $checkoutPath, 'rev-parse', 'HEAD')) $initialCommit 'Dry run preserves HEAD'
    Assert-Equal (Invoke-TestGit @('-C', $checkoutPath, 'diff', '--cached', '--name-only')) $null 'Dry run preserves index'
    Assert-Equal (Invoke-TestGit @('-C', $checkoutPath, 'branch', '--show-current')) 'main' 'Dry run preserves branch'
    Invoke-TestSync | Out-Null
    $firstCommit = Invoke-TestGit @('-C', $checkoutPath, 'rev-parse', 'HEAD')
    Assert-Equal (Invoke-TestGit @('--git-dir', $remotePath, 'rev-parse', 'main')) $firstCommit 'Remote main receives commit'
    Assert-Equal (Invoke-TestGit @('-C', $checkoutPath, 'branch', '--show-current')) 'main' 'Local checkout returns to main'
    $syncBranch = Invoke-TestGit @('-C', $checkoutPath, 'for-each-ref', '--format=%(refname:short)', 'refs/heads/sync/')
    Assert-Equal (Invoke-TestGit @('--git-dir', $remotePath, 'rev-parse', $syncBranch)) $firstCommit 'Problem branch receives same commit'
    Assert-Equal (Invoke-TestGit @('-C', $checkoutPath, 'ls-files', 'outside.txt')) $null 'Out-of-scope file remains untracked'
    Assert-Equal (Invoke-TestGit @('-C', $checkoutPath, 'ls-files', 'GITHUB_SYNC.md')) $null 'Tracked optional-file deletion is synchronized'
    Assert-Equal (Invoke-TestGit @('-C', $checkoutPath, 'log', '-1', '--format=%s')) 'Review source change; literal $(do-not-execute)' 'Message is kept literally'
    Invoke-TestSync | Out-Null
    Assert-Equal (Invoke-TestGit @('-C', $checkoutPath, 'rev-parse', 'HEAD')) $firstCommit 'No-change sync creates no commit'

    Invoke-TestGit @('-C', $checkoutPath, 'add', 'outside.txt') | Out-Null
    Invoke-TestSync -ShouldFail | Out-Null
    Assert-Equal (Invoke-TestGit @('-C', $checkoutPath, 'diff', '--cached', '--name-only')) 'outside.txt' 'Staged work is preserved on refusal'
    Invoke-TestGit @('-C', $checkoutPath, 'restore', '--staged', 'outside.txt') | Out-Null

    [IO.File]::WriteAllText($sourcePath, 'upload blocked by server')
    $hookPath = Join-Path $remotePath 'hooks/update'
    $hookContent = '#!/bin/sh' + "`n" + 'if [ "$1" = "refs/heads/main" ]; then exit 1; fi' + "`nexit 0`n"
    [IO.File]::WriteAllText($hookPath, $hookContent, [Text.UTF8Encoding]::new($false))
    Invoke-TestSync -ShouldFail | Out-Null
    $pendingBranch = Invoke-TestGit @('-C', $checkoutPath, 'branch', '--show-current')
    $pendingCommit = Invoke-TestGit @('-C', $checkoutPath, 'rev-parse', 'HEAD')
    Assert-Equal (Invoke-TestGit @('-C', $checkoutPath, 'rev-parse', 'main')) $firstCommit 'Rejected push keeps local main'
    Assert-Equal (Invoke-TestGit @('--git-dir', $remotePath, 'rev-parse', 'main')) $firstCommit 'Rejected push keeps remote main'
    Assert-Equal (Invoke-TestGit @('--git-dir', $remotePath, 'for-each-ref', '--format=%(refname:short)', "refs/heads/$pendingBranch")) $null 'Atomic rejection does not publish problem branch'
    Remove-Item -LiteralPath $hookPath
    Invoke-TestSync | Out-Null
    Assert-Equal (Invoke-TestGit @('-C', $checkoutPath, 'rev-parse', 'HEAD')) $pendingCommit 'Retry reuses pending commit'
    Assert-Equal (Invoke-TestGit @('--git-dir', $remotePath, 'rev-parse', 'main')) $pendingCommit 'Retry publishes main'
    Assert-Equal (Invoke-TestGit @('-C', $checkoutPath, 'branch', '--show-current')) 'main' 'Retry returns to main'

    Invoke-TestGit @('-C', $checkoutPath, 'remote', 'set-url', 'origin', (Join-Path $testRoot 'wrong-repository.git')) | Out-Null
    Invoke-TestSync -ShouldFail | Out-Null
    Assert-Equal (Invoke-TestGit @('-C', $checkoutPath, 'rev-parse', 'HEAD')) $pendingCommit 'Wrong repository is rejected without a commit'
    Invoke-TestGit @('-C', $checkoutPath, 'remote', 'set-url', 'origin', $remotePath) | Out-Null

    Invoke-TestGit @('-C', $checkoutPath, 'switch', '-c', 'work-in-progress') | Out-Null
    [IO.File]::WriteAllText($sourcePath, 'unfinished branch work')
    Invoke-TestSync -ShouldFail | Out-Null
    Assert-Equal (Invoke-TestGit @('-C', $checkoutPath, 'branch', '--show-current')) 'work-in-progress' 'Other branch work is preserved'
    Invoke-TestGit @('-C', $checkoutPath, 'switch', 'main') | Out-Null

    Invoke-TestGit @('clone', $remotePath, $otherCheckout) | Out-Null
    Invoke-TestGit @('-C', $otherCheckout, 'config', 'user.name', 'Upstream test') | Out-Null
    Invoke-TestGit @('-C', $otherCheckout, 'config', 'user.email', 'upstream@example.invalid') | Out-Null
    [IO.File]::WriteAllText((Join-Path $otherCheckout 'SMART_CAR_2026/source.txt'), 'independent upstream edit')
    Invoke-TestGit @('-C', $otherCheckout, 'add', '.') | Out-Null
    Invoke-TestGit @('-C', $otherCheckout, 'commit', '-m', 'upstream edit') | Out-Null
    Invoke-TestGit @('-C', $otherCheckout, 'push', 'origin', 'main') | Out-Null
    Invoke-TestSync -ShouldFail | Out-Null
    Assert-Equal (Invoke-TestGit @('-C', $checkoutPath, 'rev-parse', 'HEAD')) $pendingCommit 'Independent upstream prevents a local commit'
    Assert-Equal ([IO.File]::ReadAllText($sourcePath)) 'unfinished branch work' 'Independent upstream does not overwrite work'
    Write-Output "PASS $checks sync integration checks (local repositories only)"
} finally {
    # Check the generated absolute path before deleting this disposable test tree.
    $resolvedTestRoot = [IO.Path]::GetFullPath($testRoot)
    $temporaryRoot = [IO.Path]::GetFullPath([IO.Path]::GetTempPath()).TrimEnd('\') + '\'
    if (!$resolvedTestRoot.StartsWith($temporaryRoot, [StringComparison]::OrdinalIgnoreCase) -or
        !(Split-Path -Leaf $resolvedTestRoot).StartsWith('loongson-sync-test-')) {
        throw "Refusing to delete unexpected test path: $resolvedTestRoot"
    }
    Remove-Item -LiteralPath $resolvedTestRoot -Recurse -Force
}
