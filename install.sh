#!/bin/bash
# KAST installer
#
# Usage (on the machine running klippy, e.g. your Pi):
#   wget -O - https://raw.githubusercontent.com/DonaldLSayers/Klipper-Automated-Sensorless-Tuning/main/install.sh | bash
#
# Re-running this script is safe, it only creates/updates things that
# are missing or out of date.
#
# Override any of these via environment variables if your install
# doesn't use the usual MainsailOS/Fluidd-style layout, e.g.:
#   KLIPPER_DIR=/home/pi/klipper ./install.sh

set -e

KAST_DIR="${KAST_DIR:-$HOME/kast}"
KLIPPER_DIR="${KLIPPER_DIR:-$HOME/klipper}"
KLIPPER_VENV="${KLIPPER_VENV:-$HOME/klippy-env}"
PRINTER_DATA="${PRINTER_DATA:-$HOME/printer_data}"
CONFIG_DIR="${CONFIG_DIR:-$PRINTER_DATA/config}"
BACKUP_DIR="${BACKUP_DIR:-$HOME/kast-backups}"
REPO_URL="https://github.com/DonaldLSayers/Klipper-Automated-Sensorless-Tuning.git"

info() { echo "[kast-install] $1"; }
warn() { echo "[kast-install] WARNING: $1" >&2; }

if [ "$EUID" -eq 0 ]; then
    echo "Don't run this as root, run it as the user klipper runs as." >&2
    exit 1
fi

if ! command -v python3 >/dev/null 2>&1; then
    echo "python3 not found, is Klipper installed?" >&2
    exit 1
fi

if ! systemctl list-units --all 2>/dev/null | grep -q "klipper"; then
    warn "couldn't find a klipper systemd service, continuing anyway"
fi

# 0. Back up the whole config folder before touching anything. This
# installer only appends/symlinks, it never overwrites, but a backup
# costs nothing and means a bad merge or a fat-fingered edit later is
# never more than a copy away from undone.
if [ -d "$CONFIG_DIR" ]; then
    mkdir -p "$BACKUP_DIR"
    BACKUP_FILE="$BACKUP_DIR/config-backup-$(date +%Y%m%d-%H%M%S).tar.gz"
    tar -czf "$BACKUP_FILE" -C "$(dirname "$CONFIG_DIR")" "$(basename "$CONFIG_DIR")"
    info "backed up $CONFIG_DIR to $BACKUP_FILE"
else
    warn "no config dir found at $CONFIG_DIR yet, skipping backup"
fi

# 1. Clone or update the repo
if [ -d "$KAST_DIR/.git" ]; then
    info "updating existing checkout at $KAST_DIR"
    git -C "$KAST_DIR" pull --ff-only
else
    info "cloning into $KAST_DIR"
    git clone "$REPO_URL" "$KAST_DIR"
fi

# 2. Symlink the extras module into Klipper
if [ ! -d "$KLIPPER_DIR/klippy/extras" ]; then
    echo "Klipper not found at $KLIPPER_DIR (set KLIPPER_DIR if it's elsewhere)" >&2
    exit 1
fi

EXTRAS_LINK="$KLIPPER_DIR/klippy/extras/kast.py"
if [ -e "$EXTRAS_LINK" ] && [ ! -L "$EXTRAS_LINK" ]; then
    warn "$EXTRAS_LINK already exists and isn't a symlink, backing it up to kast.py.bak"
    mv "$EXTRAS_LINK" "$EXTRAS_LINK.bak"
fi
ln -sf "$KAST_DIR/klippy/extras/kast.py" "$EXTRAS_LINK"
info "linked klippy/extras/kast.py"

# 3. Symlink the macros into printer_data/config and make sure they're included
if [ ! -d "$CONFIG_DIR" ]; then
    echo "printer config dir not found at $CONFIG_DIR (set PRINTER_DATA if it's elsewhere)" >&2
    exit 1
fi

MACRO_LINK="$CONFIG_DIR/kast-macros.cfg"
ln -sf "$KAST_DIR/macros/kast.cfg" "$MACRO_LINK"
info "linked macros/kast.cfg -> $MACRO_LINK"

PRINTER_CFG="$CONFIG_DIR/printer.cfg"
if [ -f "$PRINTER_CFG" ] && ! grep -q "kast-macros.cfg" "$PRINTER_CFG"; then
    # Klipper requires its auto-generated "#*# <---- SAVE_CONFIG ---->"
    # block to be the last thing in the file (that's where saved
    # calibration data like bed mesh / PID / Z-offset lives). Appending
    # after it breaks that and corrupts the saved data, so insert the
    # include before that marker if it exists, otherwise it's safe to
    # just append at the end.
    if grep -q '^#\*# <---' "$PRINTER_CFG"; then
        awk -v inc='[include kast-macros.cfg]' '
            !done && /^#\*# <---/ { print inc; print ""; done=1 }
            { print }
        ' "$PRINTER_CFG" > "$PRINTER_CFG.kast.tmp" && mv "$PRINTER_CFG.kast.tmp" "$PRINTER_CFG"
        info "inserted [include kast-macros.cfg] before the SAVE_CONFIG block in printer.cfg"
    else
        printf '\n[include kast-macros.cfg]\n' >> "$PRINTER_CFG"
        info "added [include kast-macros.cfg] to printer.cfg"
    fi
elif [ ! -f "$PRINTER_CFG" ]; then
    warn "no printer.cfg found at $PRINTER_CFG, add '[include kast-macros.cfg]' yourself"
fi

# 4. Install matplotlib into Klipper's venv (best-effort, needed for auto-plotting only)
if [ -x "$KLIPPER_VENV/bin/pip" ]; then
    info "installing matplotlib into $KLIPPER_VENV (for auto-plotting graphs)"
    if ! "$KLIPPER_VENV/bin/pip" install -q matplotlib; then
        warn "matplotlib install failed, KAST still works, it just won't auto-render PNGs"
    fi
else
    warn "couldn't find Klipper's venv at $KLIPPER_VENV, skipping matplotlib install"
fi

# 5. Register with Moonraker's update manager, if moonraker.conf exists
MOONRAKER_CFG="$CONFIG_DIR/moonraker.conf"
if [ -f "$MOONRAKER_CFG" ] && ! grep -q "\[update_manager kast\]" "$MOONRAKER_CFG"; then
    cat >> "$MOONRAKER_CFG" <<EOF

[update_manager kast]
type: git_repo
path: $KAST_DIR
origin: $REPO_URL
primary_branch: main
managed_services: klipper
EOF
    info "added [update_manager kast] to moonraker.conf"
fi

info "done. Now add a [kast] section to printer.cfg (see docs/example-printer.cfg), then restart Klipper."
