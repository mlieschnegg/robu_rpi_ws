# Managed ROBU workspace updates

The Ubuntu 24.04 autostart loads its functions before updating itself and uses
`flock` to prevent concurrent autostart runs. The lock and latest update log live
outside the checkout in `${XDG_STATE_HOME:-$HOME/.local/state}/robu-autostart`.
The log is overwritten on each run, not appended indefinitely.

Only `~/work/.robu` is centrally managed: after a successful fetch from
`https://github.com/mlieschnegg/robu_rpi_ws.git`, its tracked files and local
commits are reset to `origin/main`. Do not store student work there. Untracked
files are not generally cleaned (Git may remove ones obstructing checkout).
Other RoboCup repositories retain their previous update behavior.

A successful-build commit marker causes a build on changed/restored source,
missing installation, initial migration, or an earlier failed/interrupted build.
ROS is sourced before building. A failed fetch never resets or reclones a healthy
checkout; the current installation remains usable. An unfinished build can still
be retried offline. A build failure stops this autostart rather than loading a
possibly partially updated installation.

`git fsck --full` checks the managed repository at startup. Recognized corruption,
a missing `.git` directory or an unusable HEAD triggers a replacement clone.
Unknown check errors (for example permissions) stop the update for investigation.
A failed fetch alone is not evidence of corruption. Permission to discard local
changes applies only to the managed ROBU checkout, not student repositories.

Repair first clones and checks a temporary sibling directory, then moves it into
the final location and builds there. On clone failure the existing directory is
retained. The preceding checkout is kept as `~/work/.robu.recovery`; a subsequent
successful repair replaces that recovery copy, so backups do not accumulate.
An interruption between the directory renames is recovered on the next start.
An abrupt power-off during cloning can leave a `.robu.clone.*` temporary folder;
inspect/remove such leftovers with the autostart stopped. Git fsync is enabled,
but cannot guarantee recovery from virtual disk/filesystem damage or a full disk.

An old `.git/index.lock` is not automatically deleted: it can belong to another
Git process. Investigate the log before removing locks. Existing VS Code caches
are no longer deleted blindly while the editor may be running.

Run integration checks with `bash autostart/test_update_robu.sh`. They use a local
bare remote and a stub build command; no ROS installation or GitHub is required.
The Linux `flock` invocation and a real ROS build still need testing in the image.

For initial deployment, update the complete repository so both `autostart.sh`
and `update_robu.sh` exist before launching the new script.
