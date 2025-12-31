# Contributing to Team 18650's SDK Repository

This guide is for team members contributing to our FTC robot code.

## Getting Started

1. Clone this repository (not the official FtcRobotController)
2. Open the project in Android Studio Ladybug (2024.2) or later
3. Make your changes in the `TeamCode` folder
4. Test on the robot before committing

## Workflow

### For Most Changes
1. Pull the latest changes: `git pull`
2. Make your changes
3. Test on the robot
4. Commit with a descriptive message: `git commit -m "Add arm preset positions"`
5. Push: `git push`

### For Experimental Features
If you're working on something experimental that might break things:
1. Create a branch: `git checkout -b my-feature`
2. Make your changes and test
3. Push your branch: `git push -u origin my-feature`
4. Ask a mentor to review before merging

## Commit Messages

Write clear commit messages that explain *what* you changed:
- **Good:** "Add autonomous routine for left start position"
- **Good:** "Fix arm motor direction"
- **Bad:** "Update code"
- **Bad:** "Fixed stuff"

## Where to Put Code

All team code goes in:
```
TeamCode/src/main/java/org/firstinspires/ftc/teamcode/
```

Do not modify files outside of `TeamCode` unless you have a specific reason and have discussed it with a mentor.

## Need Help?

- Ask a mentor or experienced team member
- Check the [FTC Documentation](https://ftc-docs.firstinspires.org/index.html)
- Visit the [FTC Community Forum](https://ftc-community.firstinspires.org/)

## Reporting Bugs in the Official SDK

If you find a bug in the FTC SDK itself (not our team code), report it to the official repository:
https://github.com/FIRST-Tech-Challenge/FtcRobotController/issues
