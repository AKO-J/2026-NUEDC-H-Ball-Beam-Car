#!/bin/zsh
# Flash an LP-MSPM0G3507 through its onboard TI XDS110 debugger.
# The default action verifies but deliberately does not start the target.
# Pass --run after --yes only for safe, isolated diagnostics.

set -euo pipefail

script_dir=${0:A:h}
script_name=${0:t}
dslite=/Applications/TI/ccs/ccs_base/DebugServer/bin/DSLite
config_file="$script_dir/targetConfigs/mspm0g3507_xds110.ccxml"
default_image="$script_dir/build/line_follow_test.out"

usage() {
    print "Usage: $script_name --yes [--run] [firmware.out]"
    print ""
    print "Default firmware: $default_image"
    print "Without --run this command erases, flashes and verifies the target, but does not run it."
}

if [[ ${1:-} != --yes ]]; then
    usage
    exit 2
fi
shift

run_after_flash=false
if [[ ${1:-} == --run ]]; then
    run_after_flash=true
    shift
fi

if [[ $# -gt 1 ]]; then
    usage
    exit 2
fi

image=${1:-$default_image}

if [[ ! -x $dslite ]]; then
    print -u2 "DSLite was not found: $dslite"
    exit 1
fi

if [[ ! -f $config_file ]]; then
    print -u2 "XDS110 target configuration is missing: $config_file"
    exit 1
fi

if [[ ! -f $image ]]; then
    print -u2 "Firmware image was not found: $image"
    exit 1
fi

print "Safety check: VM should be disconnected or the drive wheels must be suspended."
print "Flashing and verifying through the onboard XDS110: $image"

tmp_appdata=$(mktemp -d /private/tmp/mspm0_xds110.XXXXXX)
trap 'rm -rf "$tmp_appdata"' EXIT

flash_args=(flash --config="$config_file" --flash --verify --verbose)
if [[ $run_after_flash == true ]]; then
    flash_args+=(--run)
fi

arch -x86_64 env "TI_APPDATA_DIR=$tmp_appdata" "$dslite" "${flash_args[@]}" "$image"

if [[ $run_after_flash == true ]]; then
    print "Flash verification completed. The target was started."
else
    print "Flash verification completed. The target was not started automatically."
fi
