

## 🧰 Step-by-step GitHub setup on Ubuntu

### 1️⃣ Install Git

Make sure Git is installed:

```bash
sudo apt update
sudo apt install git -y
```

Check it:

```bash
git --version
```

---

### 2️⃣ Configure your Git identity

Tell Git who you are (this information will appear on your commits):

```bash
git config --global user.name "Epik-Robotik"
git config --global user.email "portal.epik@gmail.com"
```

Verify:

```bash
git config --list
```

You should see:

```
user.name=Epik-Robotik
user.email=portal.epik@gmail.com
```

---

### 3️⃣ Generate an SSH key

This lets you connect securely to GitHub without typing a password every time.

Run:

```bash
ssh-keygen -t ed25519 -C "portal.epik@gmail.com"
```

When prompted for a file location, press **Enter** (default is fine).
You can leave the passphrase empty or add one for extra security.

---

### 4️⃣ Start the SSH agent and add your key

```bash
eval "$(ssh-agent -s)"
ssh-add ~/.ssh/id_ed25519
```

---

### 5️⃣ Add your SSH key to your GitHub account

Get your public key:

```bash
cat ~/.ssh/id_ed25519.pub
```

Copy the entire output (it starts with `ssh-ed25519`).

Then go to:

🔗 [https://github.com/settings/keys](https://github.com/settings/keys)
→ Click **“New SSH key”**
→ Give it a title like _Ubuntu Laptop_
→ Paste your key
→ Click **Add SSH key**

---

### 6️⃣ Test your connection

Now confirm the setup works:

```bash
ssh -T git@github.com
```

You should see something like:

```
Hi Epik-Robotik! You've successfully authenticated, but GitHub does not provide shell access.
```

---

### 7️⃣ Clone or push repositories

Now you can use SSH links to work with your repos, e.g.:

```bash
git clone git@github.com:Epik-Robotik/repo-name.git
```

To push commits:

```bash
git add .
git commit -m "Initial commit"
git push origin main
```

---

✅ Done! Your Ubuntu terminal is now **fully linked** to your GitHub account `Epik-Robotik`.


