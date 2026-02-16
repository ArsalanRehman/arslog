# Clean Uninstall & Reinstall Guide — MetaTrader 5 (MT5) on Ubuntu using Wine

---

## 1️⃣ Kill All Wine Processes

Stop any running Wine services:

```bash
wineserver -k
pkill -f wine
```

Verify nothing is running:

```bash
ps aux | grep wine
```

---

## 2️⃣ Remove MT5 Installation Files

Delete MT5 program files:

```bash
rm -rf ~/.wine/drive_c/Program\ Files/MetaTrader*
rm -rf ~/.wine/drive_c/Program\ Files\ \(x86\)/MetaTrader*
```

Remove MT5 user data:

```bash
rm -rf ~/.wine/drive_c/users/$USER/AppData/Roaming/MetaQuotes
rm -rf ~/.wine/drive_c/users/$USER/AppData/Local/MetaQuotes
```

---

## 3️⃣ Delete Entire Wine Prefix (Recommended Clean Reset)

⚠️ This removes ALL Windows apps installed in Wine.

```bash
rm -rf ~/.wine
```

---

## 4️⃣ Reinstall Wine Cleanly (Optional but Recommended)

Remove Wine:

```bash
sudo apt purge wine* -y
sudo apt autoremove -y
```

Reinstall fresh:

```bash
sudo dpkg --add-architecture i386
sudo apt update
sudo apt install wine-staging winetricks -y
```

Verify installation:

```bash
wine --version
```

---

## 5️⃣ Create Fresh Wine Environment

Initialize Wine:

```bash
winecfg
```

- Set Windows version to **Windows 10**
- Close

---

## 6️⃣ Download MT5 Installer

```bash
wget https://download.mql5.com/cdn/web/metaquotes.software.corp/mt5/mt5setup.exe -O mt5setup.exe
```

Or download via browser (usually saved in `~/Downloads`).

---

## 7️⃣ Install MT5

If in Downloads folder:

```bash
cd ~/Downloads
wine mt5setup.exe
```

Or using full path:

```bash
wine ~/Downloads/mt5setup.exe
```

---

## 8️⃣ Run MT5 in Portable Mode (Recommended)

Navigate to installation folder:

```bash
cd ~/.wine/drive_c/Program\ Files/MetaTrader\ 5
wine terminal64.exe /portable
```

Portable mode prevents background Wine service issues.

---

## 9️⃣ Optional Performance Improvements

Install DXVK and fonts:

```bash
winetricks dxvk
winetricks corefonts
```

---

## 🔟 Advanced (Recommended) — Use Dedicated Wine Prefix

Create isolated prefix:

```bash
WINEPREFIX=~/mt5wine winecfg
```

Install MT5 in that prefix:

```bash
WINEPREFIX=~/mt5wine wine mt5setup.exe
```

Run MT5:

```bash
WINEPREFIX=~/mt5wine wine terminal64.exe /portable
```

---

# Troubleshooting

If CPU usage increases over time:

- Remove Expert Advisors (EAs)
- Reduce number of charts
- Disable auto-updates
- Restart Wine using `wineserver -k`

---

# Done ✅

You now have a clean MT5 + Wine setup on Ubuntu.
