If you want a **system-wide, timestamped log of every command/app executed**, the most reliable approach is:

## Option A (recommended): auditd — log every `execve`

1. Install + enable

```bash
sudo apt update
sudo apt install auditd audispd-plugins
sudo systemctl enable --now auditd
```

2. Add exec rules (choose ONE of these):

* **Interactive users only** (safer noise level):

```bash
sudo tee /etc/audit/rules.d/exec-log.rules >/dev/null <<'EOF'
-a always,exit -F arch=b64 -S execve,execveat -F auid>=1000 -F auid!=4294967295 -k exec_log
-a always,exit -F arch=b32 -S execve,execveat -F auid>=1000 -F auid!=4294967295 -k exec_log
EOF
```

* **Everything on the box** (very noisy, lots of daemons):

```bash
sudo tee /etc/audit/rules.d/exec-log.rules >/dev/null <<'EOF'
-a always,exit -F arch=b64 -S execve,execveat -k exec_log
-a always,exit -F arch=b32 -S execve,execveat -k exec_log
EOF
```

3. Load rules

```bash
sudo augenrules --load
sudo systemctl restart auditd
```

4. Query logs (timestamps included)

```bash
# Everything today, human-readable:
sudo ausearch -k exec_log -ts today -i | less

# Show only command/path/uid/ppid summaries:
sudo aureport -x -i

# Filter by user/app/time:
sudo ausearch -k exec_log -ua <username> -ts 2025-08-27 00:00:00 -te now -i
sudo ausearch -k exec_log -x /usr/bin/python3 -i
```

* Logs live in `/var/log/audit/audit.log`.
* `auid` tracks the original user even after `sudo`, so you keep attribution.

5. Make it persist from early boot (optional but good)

* Add `audit=1` to GRUB kernel cmdline (`/etc/default/grub` → `GRUB_CMDLINE_LINUX="audit=1"`, then `sudo update-grub` and reboot).

6. Tune retention (important)
   Edit `/etc/audit/auditd.conf`:

```
max_log_file = 200        # MB per file
num_logs = 10             # how many to keep
max_log_file_action = ROTATE
space_left_action = SYSLOG
```

Then `sudo systemctl restart auditd`.

> This captures **GUI apps too** (every process start is an `execve`), including Snap/Flatpak launches.

---

## Option B (complement): process accounting (`acct`) — adds finish time & CPU

```bash
sudo apt install acct
sudo accton on
# Per-exec records with (finish) time:
lastcomm | head
# Summaries:
sa -m
```

`acct` is lightweight but stores **command names (truncated) and no full args**. Use it alongside auditd if you want both **start** (auditd) and **finish** times (acct).

---

## Shell history timestamps (per-user, optional)

For Bash:

```bash
echo 'HISTTIMEFORMAT="%F %T "' >> ~/.bashrc
echo 'PROMPT_COMMAND="history -a; $PROMPT_COMMAND"' >> ~/.bashrc
source ~/.bashrc
```

This gives per-user command history with timestamps, but **auditd** is the authoritative system-wide record.

---

## Notes & gotchas

* Expect **a lot** of data with auditd (especially “log everything”). Start with the interactive-users rules.
* Command-line **arguments** will be recorded; avoid passing secrets via CLI.
* To stop later: `sudo rm /etc/audit/rules.d/exec-log.rules && sudo augenrules --load`.

If you tell me whether you want **only human activity** vs **literally everything**, I can drop in the exact rule set and a couple of ready-made `ausearch`/`aureport` commands tailored for you.
