#!/bin/zsh
# Run the formal LF04 firmware flash inside the logged-in macOS Terminal session.
# This is a bridge for automation environments that cannot access the XDS110 USB probe.

set -u
set -o pipefail

script_dir=${0:A:h}
log_file="$script_dir/build/last_terminal_flash.log"
firmware="$script_dir/build/line_follow_test.out"

mkdir -p "$script_dir/build"

{
    print "=== MSPM0G3507 formal firmware flash ==="
    print "Started: $(date '+%Y-%m-%d %H:%M:%S')"
    print "Firmware: $firmware"
    print ""

    if [[ ! -f "$firmware" ]]; then
        print -u2 "Firmware image was not found. Run 'make line-follow' first."
        exit 1
    fi

    "$script_dir/flash_xds110.sh" --yes "$firmware"
    flash_result=$?

    print ""
    if [[ $flash_result -eq 0 ]]; then
        print "RESULT: SUCCESS"
        print "The firmware was verified and was not started automatically."
    else
        print "RESULT: FAILED ($flash_result)"
    fi
    print "Finished: $(date '+%Y-%m-%d %H:%M:%S')"
    exit $flash_result
} 2>&1 | tee "$log_file"

exit ${pipestatus[1]}
