


# 🎧 Audio Input/Output Setup Guide (Linux)

This guide ensures your system always uses:

- **Output:** Bluetooth Headset (A2DP - High Fidelity)
- **Input:** USB Microphone (SK-30)

---

## ✅ Desired Configuration

| Setting        | Device                          |
|----------------|---------------------------------|
| Output Device  | Bluetooth Headset               |
| Output Mode    | High Fidelity Playback (A2DP)   |
| Input Device   | Microphone - SK-30              |

---

## ⚙️ Step 1: Set Devices in System Settings

Go to:

**Settings → Sound**

Set the following:

- **Output Device** → Your Bluetooth headset  
- **Configuration** → `High Fidelity Playback (A2DP Sink)`  
- **Input Device** → `Microphone - SK-30`  

---

## 🔧 Step 2: Install Advanced Sound Control

Install `pavucontrol` for better control:

```bash
sudo apt install pavucontrol
````

Launch it:

```bash
pavucontrol
```

---

## 🎚️ Step 3: Configure Defaults in pavucontrol

### Output Devices Tab

* Select your Bluetooth headset
* Click **"Set as fallback"** (green checkmark)

### Input Devices Tab

* Select **Microphone - SK-30**
* Click **"Set as fallback"**

### Configuration Tab

* Set Bluetooth headset profile to:

  * **High Fidelity Playback (A2DP Sink)**

---

## ⚠️ Common Issue: Bluetooth Switching Modes

Bluetooth headsets have two modes:

| Mode    | Quality | Mic | Notes                 |
| ------- | ------- | --- | --------------------- |
| A2DP    | High    | ❌   | Best for audio output |
| HSP/HFP | Low     | ✅   | Enables headset mic   |

### Problem

When apps try to use the headset mic, the system switches to **low-quality mode (HSP/HFP)**.

---

## 🛠️ Fix: Prevent Mode Switching

### Option 1 (Recommended)

* Always use **SK-30 mic**
* In apps (Discord, Zoom, OBS):

  * Set mic manually to **SK-30**
  * Avoid using "Default"

### Option 2

In `pavucontrol → Input Devices`:

* Mute or ignore the headset microphone if it appears

---

## 🔄 Step 4: Make It Persistent

If settings reset after reboot:

* Open `pavucontrol`
* Ensure both devices are set as **fallback**
* Reconnect Bluetooth headset after login if needed

---

## 💡 Pro Tips

* Turn on your USB mic before connecting Bluetooth
* Avoid using headset mic entirely
* Always verify settings inside apps (they override system defaults)

---

## ✅ Final Result

* 🎧 High-quality audio through headset (A2DP)
* 🎤 Clear input from SK-30 microphone
* 🚫 No unwanted switching to low-quality Bluetooth mode

---
