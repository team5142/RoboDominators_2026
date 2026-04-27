# Laptop Setup Checklist

Complete every step before your first programming session.
Check each box as you finish it. Ask a mentor if you get stuck.

---

## Official WPILib References

These are the official docs — bookmark them. They cover everything in much more depth than this guide.

- Getting Started: https://docs.wpilib.org/en/stable/docs/zero-to-robot/introduction.html
- Installing WPILib: https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/wpilib-setup.html
- VS Code overview: https://docs.wpilib.org/en/stable/docs/software/vscode-overview/index.html
- WPILib Java API: https://github.wpilib.org/allwpilib/docs/release/java/index.html
- Command-based programming: https://docs.wpilib.org/en/stable/docs/software/commandbased/index.html

---

## Step 1 — Install WPILib 2026

WPILib is the official FRC programming library. It includes VS Code, Java, and all the tools you need.

1. Go to: https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/wpilib-setup.html
   (The page links directly to the correct installer for your year — follow the instructions there.)
2. Download the installer for your OS:
   - Windows: `WPILib_Windows-2026.x.x.iso`
   - Mac: `WPILib_macOS-2026.x.x.dmg`
3. Run the installer. When asked, choose **"Everything"** for install type.
4. When it finishes, open **WPILib VS Code** from your Start menu or Applications.
   - This is a separate VS Code from any you may already have installed — use this one for robot code.

---

## Step 2 — Install Git

Git is how we track code changes and share work between team members.

1. Go to: https://git-scm.com/downloads
2. Download and run the installer for your OS.
3. During Windows install, leave all defaults as-is and click Next through everything.
4. When finished, open a terminal:
   - Windows: search "PowerShell" in Start menu
   - Mac: open Terminal from Applications
5. Type this and press Enter:
   ```
   git --version
   ```
   You should see something like `git version 2.x.x`. If you do, Git is installed correctly.

---

## Step 3 — Create a GitHub Account

GitHub is where the team stores all robot code online.

1. Go to: https://github.com
2. Click **Sign up** and create a free account.
3. Use a username you are comfortable sharing with the team (your name is fine).
4. Tell a mentor your GitHub username so you can be added to the team organization.

---

## Step 4 — Configure Git on Your Laptop

Tell Git who you are so your commits are labeled with your name.

Open PowerShell (Windows) or Terminal (Mac) and run these two commands,
replacing the name and email with yours:

```
git config --global user.name "Your Name"
git config --global user.email "your@email.com"
```

No output means it worked.

---

## Step 5 — Clone the Robot Code

"Cloning" downloads a copy of the repository to your laptop.

1. Open PowerShell or Terminal.
2. Navigate to where you want to store the project. For example:
   ```
   cd C:\Users\YourName\Documents
   ```
3. Clone the repo:
   ```
   git clone https://github.com/team5142/RoboDominators_2026.git
   ```
4. Move into the folder:
   ```
   cd RoboDominators_2026
   ```

---

## Step 6 — Open the Project in WPILib VS Code

1. Open **WPILib VS Code** (not regular VS Code).
2. Click **File → Open Folder**.
3. Navigate to the `RoboDominators_2026` folder you just cloned and click **Select Folder**.
4. VS Code will ask if you trust the folder — click **Yes**.
5. Wait for the Java extension to finish loading (bottom status bar will stop spinning).

---

## Step 7 — Verify the Build Works

1. In VS Code, open the Terminal: **View → Terminal**.
2. Run:
   ```
   .\gradlew.bat compileJava
   ```
   (Mac/Linux: `./gradlew compileJava`)
3. You should see `BUILD SUCCESSFUL` at the end.
   If you see errors, ask a mentor before continuing.

---

## You Are Ready

When all steps are done, move on to **docs/GIT_GUIDE.md** — that is Task 1.
