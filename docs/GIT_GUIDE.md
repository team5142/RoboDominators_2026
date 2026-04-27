# Task 1 — Git: Your Personal Branch

Git is how programmers track every change they make to code.
Instead of saving over files, Git takes snapshots called **commits**.
You can always go back to any previous snapshot.

This guide has two parts:
- Part A: Understand the key commands
- Part B: Type them yourself to set up your branch

---

## Part A — The Commands You Need

### `git status`
Shows what files you have changed since your last commit.
Run this constantly — it is the most useful command in Git.
```
git status
```

### `git branch`
Lists all branches. The one with `*` next to it is where you are now.
```
git branch
```

### `git checkout -b <name>`
Creates a new branch and switches to it.
A branch is your own workspace — changes here do not affect anyone else.
```
git checkout -b student-yourname
```

### `git add <file>`
Stages a file — tells Git "include this file in my next commit."
```
git add src/main/java/frc/robot/subsystems/SpindexerSubsystem.java
```
Or stage everything you changed at once:
```
git add -A
```

### `git commit -m "message"`
Takes a snapshot of everything you staged.
The message describes what you changed — be specific, not "stuff" or "changes."
```
git commit -m "Spindexer: add spinForward and stop methods"
```

### `git push`
Uploads your commits to GitHub so they are saved online and visible to mentors.
The first time you push a new branch, Git will tell you to run a longer command —
just copy and paste exactly what it prints.
```
git push
```

### `git log --oneline`
Shows a short history of your commits.
```
git log --oneline
```

### `git diff`
Shows exactly what changed in your files since your last commit, line by line.
Lines starting with `+` were added. Lines starting with `-` were removed.
```
git diff
```

---

## Part B — Set Up Your Branch

Open PowerShell or the VS Code terminal and run each command below.
**Type them manually** — do not copy/paste. Typing builds muscle memory.

**1. Make sure you are on the student practice branch:**
```
git checkout Summer2026ProgrammingPractice
```

**2. Check the current status (should show nothing changed):**
```
git status
```

**3. Create your personal branch. Replace `yourname` with your actual first name:**
```
git checkout -b student-yourname
```

**4. Verify you are on your new branch:**
```
git branch
```
Your branch should have the `*` next to it.

**5. Open `SpindexerSubsystem.java` in VS Code and add a comment on any blank line:**
```java
// Hello from yourname
```
Save the file.

**6. Check status — Git should show the file as modified:**
```
git status
```

**7. Stage the file:**
```
git add -A
```

**8. Commit it:**
```
git commit -m "First commit - testing git"
```

**9. Push to GitHub:**
```
git push --set-upstream origin student-yourname
```

**10. Go to https://github.com/team5142/RoboDominators_2026 in your browser.**
Click **Branches** — you should see your branch listed there.

---

## Going Forward

Every time you finish a meaningful chunk of work:
1. `git add -A`
2. `git commit -m "short description of what you did"`
3. `git push`

Commit often. Small commits are easier to understand and easier to undo than large ones.

---

## Comparing Your Work to the Answer Key

The `Summer2026ProgrammingPractice-Master` branch has the complete working version of everything.
To see how your version of a file compares to the answer key:

```
git diff Summer2026ProgrammingPractice-Master -- src/main/java/frc/robot/subsystems/SpindexerSubsystem.java
```

Lines you wrote that differ from master will show up in the diff.
Use this when you are stuck or want to check your work — not as a first resort.

---

## You Are Done with Task 1

Open `SpindexerSubsystem.java` and start **Task 2**.
The task description is in the comments at the top of that file.
