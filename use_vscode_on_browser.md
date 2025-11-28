You’ve got two solid ways to run “real” VS Code in your **browser on localhost** on Ubuntu 20.04, with full file access and a terminal to run code. The easiest is **code-server**; an alternative is **OpenVSCode-Server**. Here’s the quick, safe path.

---

# Option A — code-server (recommended)

This runs VS Code as a local service, listens only on `127.0.0.1`, and lets you open any folder you own, install extensions, and use the integrated terminal.

1. Install

```bash
curl -fsSL https://code-server.dev/install.sh | sh
sudo systemctl enable --now code-server@$USER
```

Then open [http://127.0.0.1:8080](http://127.0.0.1:8080) in your browser. The first-time password is written to `~/.config/code-server/config.yaml`. ([Coder][1])

2. Set it to your liking (optional but nice)

```bash
nano ~/.config/code-server/config.yaml
# common tweaks:
# bind-addr: 127.0.0.1:8080   (local only)
# auth: password              (default)
# password: <your-long-password>
sudo systemctl restart code-server@$USER
```

(Password location and restart flow are from the official docs.) ([Coder][2])

3. Use it like desktop VS Code

* Click “Open Folder…” and choose any directory you have access to.
* Install your usual extensions (Python, C/C++, Docker, etc.).
* Use the integrated **Terminal** to run and debug your code with your system’s runtimes and tools.

> Notes
> • Keeping `bind-addr: 127.0.0.1:8080` means it’s **not exposed** on your network. Only your machine can reach it. ([Coder][3])
> • There’s also a Snap if you prefer: `sudo snap install code-server`. ([Snapcraft][4])

---

# Option B — OpenVSCode-Server (alternative)

Also VS Code in the browser, maintained by Gitpod. If you like Docker or want a pure upstream build, use this.

**Docker (quickest):**

```bash
docker run -it --name openvscode \
  -p 127.0.0.1:3000:3000 \
  -v "$HOME:/home/workspace" \
  -u "$(id -u):$(id -g)" \
  gitpod/openvscode-server:latest
```

Then visit [http://127.0.0.1:3000](http://127.0.0.1:3000) and open `/home/workspace`. (Project details and releases are on GitHub.) ([GitHub][5], [Docker Hub][6])

---

## Which should you pick?

* **Want simplest native setup, systemd service, great docs?** → **code-server.** ([Coder][1])
* **Prefer a minimal container or upstream flavor?** → **OpenVSCode-Server.** ([GitHub][5])

---

## Verify you can “execute code”

Inside the browser VS Code:

1. Open the **Terminal** (\`Ctrl+\`\`).
2. Run your tooling as usual, e.g.:

```bash
python3 --version
node -v
gcc --version
```

(Install missing runtimes with `apt` as you normally would.)

---

## (Optional) Docker way for code-server

If you really want it containerized but still local, here’s the official example (mounts your config and current folder; adjust volumes if you want more of your FS available):

```bash
mkdir -p ~/.config
docker run -it --name code-server -p 127.0.0.1:8080:8080 \
  -v "$HOME/.local:/home/coder/.local" \
  -v "$HOME/.config:/home/coder/.config" \
  -v "$PWD:/home/coder/project" \
  -u "$(id -u):$(id -g)" \
  -e "DOCKER_USER=$USER" \
  codercom/code-server:latest
```

Open [http://127.0.0.1:8080](http://127.0.0.1:8080). (Official doc snippet.) ([Coder][1])

---

If you tell me which languages you’ll run (Python, Node, C/C++, ROS, etc.), I’ll add the exact extensions/runtimes and a tiny test project so you can confirm everything’s wired up.

[1]: https://coder.com/docs/code-server/install "Install code-server: OS Instructions for VS Code | code-server Docs"
[2]: https://coder.com/docs/code-server/install?utm_source=chatgpt.com "Install code-server: OS Instructions for VS Code"
[3]: https://coder.com/docs/code-server/guide?utm_source=chatgpt.com "Securely Access & Expose code-server - Coder"
[4]: https://snapcraft.io/install/code-server/ubuntu?utm_source=chatgpt.com "Install code-server on Ubuntu using the Snap Store"
[5]: https://github.com/gitpod-io/openvscode-server?utm_source=chatgpt.com "gitpod-io/openvscode-server: Run upstream VS Code on a ..."
[6]: https://hub.docker.com/r/linuxserver/openvscode-server?utm_source=chatgpt.com "linuxserver/openvscode-server - Docker Image"
