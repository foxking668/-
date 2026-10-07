# Run after review and validation. This script does not watch file saves.
[CmdletBinding()]
param(
    [string]$Message = 'Synchronize reviewed Loongson car changes',
    [ValidatePattern('^[a-z0-9]+(?:-[a-z0-9]+)*$')]
    [string]$BranchSlug = 'update',
    [switch]$DryRun,
    # Test override; production callers should leave this default unchanged.
    [string]$ExpectedRemote = 'https://github.com/foxking668/-.git'
)

$ErrorActionPreference = 'Stop'
$repositoryRoot = Split-Path -Parent $PSScriptRoot
$syncPaths = @('SMART_CAR_2026', 'AGENTS.md', '.gitignore', 'GITHUB_SYNC.md', 'scripts')

function Invoke-RepoGit {
    param([string[]]$GitArguments)
    # Capture native failures without PowerShell 5 treating stderr as termination.
    $ErrorActionPreference = 'Continue'
    $output = @(& git -C $repositoryRoot @GitArguments 2>&1)
    $gitExitCode = $LASTEXITCODE
    if ($gitExitCode -ne 0) {
        throw "git $($GitArguments -join ' ') failed (exit $gitExitCode):`n$($output -join "`n")"
    }
    $output | ForEach-Object { $_.ToString() }
}

function Assert-NoScopedChanges {
    $changes = @(Invoke-RepoGit -GitArguments (@('status', '--porcelain', '--') + $syncPaths))
    if ($changes.Count -gt 0) {
        throw "Project files changed while synchronizing, or a pending sync has new edits. Review them before continuing:`n$($changes -join "`n")"
    }
}

function Confirm-RemoteCommit {
    param([string]$Commit, [string]$Branch)
    $refs = @(Invoke-RepoGit -GitArguments @('ls-remote', '--heads', 'origin', 'refs/heads/main', "refs/heads/$Branch"))
    foreach ($refName in @('refs/heads/main', "refs/heads/$Branch")) {
        $matchedRefs = @($refs | Where-Object {
            $fields = $_ -split '\s+'
            $fields.Count -eq 2 -and $fields[0] -eq $Commit -and $fields[1] -ceq $refName
        })
        if ($matchedRefs.Count -ne 1) { throw "Remote verification failed for $refName. Keep the local commit and retry." }
    }
}

if ([string]::IsNullOrWhiteSpace($Message)) { throw 'A non-empty change description is required.' }
if (!(Get-Command git -ErrorAction SilentlyContinue)) { throw 'Git is not installed or not on PATH.' }
if (!(Test-Path -LiteralPath (Join-Path $repositoryRoot 'SMART_CAR_2026') -PathType Container)) {
    throw 'SMART_CAR_2026 is missing. Run this script from the correct project checkout.'
}
$actualRoot = Invoke-RepoGit -GitArguments @('rev-parse', '--show-toplevel')
if ([IO.Path]::GetFullPath($actualRoot) -ne [IO.Path]::GetFullPath($repositoryRoot)) {
    throw 'The script must live in the scripts directory of the repository root.'
}
# Keep tracked deletions, but omit optional files that never existed in a checkout.
$syncPaths = @($syncPaths | Where-Object {
    (Test-Path -LiteralPath (Join-Path $repositoryRoot $_)) -or
        @(Invoke-RepoGit -GitArguments @('ls-files', '--', $_)).Count -gt 0
})
foreach ($remoteArguments in @(@('remote', 'get-url', '--all', 'origin'), @('remote', 'get-url', '--push', '--all', 'origin'))) {
    $urls = @(Invoke-RepoGit -GitArguments $remoteArguments)
    if ($urls.Count -ne 1 -or $urls[0] -cne $ExpectedRemote) {
        throw "Wrong origin URL. Expected exactly one fetch/push URL: $ExpectedRemote"
    }
}
$branch = Invoke-RepoGit -GitArguments @('branch', '--show-current')
$isRetry = $branch -cmatch '^sync/\d{8}-\d{6}-[a-f0-9]{8}-[a-z0-9]+(?:-[a-z0-9]+)*$'
if ($branch -cne 'main' -and !$isRetry) {
    throw "Current branch is '$branch'. Finish that work before synchronizing from main."
}
$stagedFiles = @(Invoke-RepoGit -GitArguments @('diff', '--cached', '--name-only'))
if ($stagedFiles.Count -gt 0) { throw 'The index already has staged changes. Commit or unstage them explicitly first.' }
$gitDirectory = Invoke-RepoGit -GitArguments @('rev-parse', '--absolute-git-dir')
foreach ($operationPath in @('MERGE_HEAD', 'CHERRY_PICK_HEAD', 'REVERT_HEAD', 'rebase-merge', 'rebase-apply')) {
    if (Test-Path -LiteralPath (Join-Path $gitDirectory $operationPath)) {
        throw "A Git operation is in progress ($operationPath). Finish it first."
    }
}
$changes = @(Invoke-RepoGit -GitArguments (@('status', '--short', '--') + $syncPaths))
if ($DryRun) {
    Write-Output "DRY RUN: $repositoryRoot -> $ExpectedRemote (main)"
    Write-Output "Message: $Message"
    $changes | Write-Output
    Write-Output 'No files, index, branches or remote refs were changed. Tests must be run before a real sync.'
    return
}
if (!$isRetry -and $changes.Count -eq 0) { Write-Output 'No project changes to synchronize.'; return }
if ($isRetry) { Assert-NoScopedChanges }

# Fetch before creating a commit; do not merge unknown upstream changes automatically.
Invoke-RepoGit -GitArguments @('fetch', 'origin', 'main') | Write-Output
$mainCommit = Invoke-RepoGit -GitArguments @('rev-parse', 'main')
$remoteMain = Invoke-RepoGit -GitArguments @('rev-parse', 'refs/remotes/origin/main')
if (!$isRetry) {
    if ($mainCommit -cne $remoteMain) { throw 'Local main differs from origin/main. Review and reconcile before uploading.' }
    $branch = 'sync/{0}-{1}-{2}' -f (Get-Date -Format 'yyyyMMdd-HHmmss'), ([Guid]::NewGuid().ToString('N').Substring(0, 8)), $BranchSlug
    Invoke-RepoGit -GitArguments @('switch', '-c', $branch) | Write-Output
    Invoke-RepoGit -GitArguments (@('add', '-A', '--') + $syncPaths) | Write-Output
    $stagedFiles = @(Invoke-RepoGit -GitArguments @('diff', '--cached', '--name-only'))
    if ($stagedFiles.Count -eq 0) { throw "No staged project changes. The empty branch $branch has been retained." }
    $messagePath = Join-Path $gitDirectory ("codex-sync-message-" + [Guid]::NewGuid().ToString('N') + '.txt')
    try {
        [IO.File]::WriteAllText($messagePath, $Message + "`n", [Text.UTF8Encoding]::new($false))
        Invoke-RepoGit -GitArguments @('commit', '--file', $messagePath) | Write-Output
    } finally {
        if (Test-Path -LiteralPath $messagePath) { Remove-Item -LiteralPath $messagePath }
    }
}
$commit = Invoke-RepoGit -GitArguments @('rev-parse', 'HEAD')
$parentCommit = Invoke-RepoGit -GitArguments @('rev-parse', 'HEAD^')
if ($parentCommit -cne $mainCommit) { throw 'The sync commit is no longer based directly on local main. Review the branch before retrying.' }
if ($remoteMain -cne $mainCommit -and $remoteMain -cne $commit) {
    throw 'Remote main changed independently. Keep this branch; review and reconcile before retrying.'
}
Assert-NoScopedChanges
# Both remote refs succeed together or fail together; no force update is used.
Invoke-RepoGit -GitArguments @('push', '--atomic', '--porcelain', 'origin', "${commit}:refs/heads/$branch", "${commit}:refs/heads/main") | Write-Output
Confirm-RemoteCommit -Commit $commit -Branch $branch
Assert-NoScopedChanges
Invoke-RepoGit -GitArguments @('switch', 'main') | Write-Output
Invoke-RepoGit -GitArguments @('merge', '--ff-only', $branch) | Write-Output
$localMain = Invoke-RepoGit -GitArguments @('rev-parse', 'HEAD')
if ($localMain -cne $commit) { throw 'Local main verification failed; the remote push already succeeded.' }
Write-Output "SUCCESS: branch=$branch commit=$commit local_main=$localMain remote_main=$commit"
Write-Output "Repository: $ExpectedRemote"

