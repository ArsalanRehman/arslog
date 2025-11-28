

**Step 1: Access the Automatic Repair Page**
* Open the Start menu and click the power icon. While holding the Shift key, click "Restart" to restart the computer [[01:07](http://www.youtube.com/watch?v=7S6OMY-9aPU&t=67)].
* Alternatively, restart your computer and as soon as the Windows logo appears, turn off the computer by unplugging it or holding down the power button. Repeat this process one more time. This action forces the computer to enter the "Automatic Repair" page.

**Step 2: Access Advanced Options**
* Once on the Automatic Repair page, click on "Advanced options."
* Select "Troubleshoot" from the menu.
* On the next screen, click on "Advanced options" again.
* Then, click on "System Image Recovery."
* A window will pop up. Click "Cancel," then "Next," and "Advanced."
* Click on "Install a driver" and then "OK."

**Step 3: Access and Rename Files**
* This will open a file explorer window. Navigate to the drive where Windows is installed (usually `C:`).
* Go to the `Windows` folder, then find and open the `System32` folder.
* In the `System32` folder, find the `utilman.exe` file. Right-click on it, select "Rename," and change its name to `utilman1.exe`.
* Next, find the `cmd.exe` file.
* Right-click on `cmd.exe`, select "Rename," and change its name to `utilman.exe`.
* Refresh the folder to confirm the changes.
* Close the file explorer and all previous windows until you can click on "Continue" to exit and return to Windows 10.

**Step 4: Reset the Password**
* When you are back on the Windows login screen, click on the "Ease of Access" icon at the bottom right corner.
* This will open the Command Prompt window.
* Type `net user [username] [new_password]` and press Enter. If your username has a space in it, you must enclose the username in double quotes, like this: `net user "Your Username" [new_password]`.
* The command prompt should display "The command completed successfully."
* Your password has now been reset, and you can log in with the new password you set.
