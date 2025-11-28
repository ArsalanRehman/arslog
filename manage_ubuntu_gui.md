Yep—turning off the desktop on Ubuntu 20.04 will free a noticeable chunk of RAM. A full GNOME session (gdm3 + Xorg + shell + helpers) typically idles in the **hundreds of MB up to \~1–1.5 GB**, depending on what’s installed. On a server, that’s nice headroom for databases, caches, etc.

Here’s the safe way to do it.

### 1) Figure out which display manager you’re using

```bash
cat /etc/X11/default-display-manager
# Likely /usr/sbin/gdm3 (GNOME) or /usr/sbin/lightdm, /usr/bin/sddm
```

### 2) Try it temporarily (current boot only)

```bash
# switch to multi-user (text) mode right now
sudo systemctl isolate multi-user.target
# OR stop just the display manager:
sudo systemctl stop gdm3   # replace with lightdm/sddm if that’s what you have
```

### 3) Make it permanent (no GUI on boot)

```bash
sudo systemctl set-default multi-user.target
sudo systemctl disable gdm3 --now   # adjust name if lightdm/sddm
```

### 4) Re-enable later (if you change your mind)

```bash
sudo systemctl set-default graphical.target
sudo systemctl enable gdm3 --now
```

### Optional: remove GUI packages to reclaim even more

Only do this if you *never* plan to use the desktop or remote desktop:

```bash
sudo apt purge ubuntu-desktop ubuntu-desktop-minimal gdm3 gnome-shell gnome-session
sudo apt autoremove
```

(Be careful not to remove things you still need; e.g., if you rely on NetworkManager for networking on that box, keep it.)

### Check the impact

Before/after RAM usage:

```bash
free -h
```

See which GUI pieces were eating memory:

```bash
ps aux | egrep 'Xorg|gdm|gnome|wayland|lightdm|sddm'
```

**Notes**

* Disabling the GUI won’t affect headless services (SSH, Docker, databases, etc.).
* If you use VNC/xrdp or GPU tools that expect X/Wayland, those will be affected.

If you tell me which desktop/display manager you have, I can tailor the exact commands.
