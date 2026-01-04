# Contributing to Team 18650's SDK Repository

This guide is for team members contributing to our FTC robot code.

## Getting Started

1. Clone this repository (not the official FtcRobotController)
2. Open the project in Android Studio Ladybug (2024.2) or later (or another IDE if you know what you are doing)
3. Make your changes in the `TeamCode` folder
4. Test on the robot before committing

## Workflow

### For Very Simple Changes
1. Check out the dev branch: `git checkout -b develop`
2. Pull the latest changes: `git pull`
3. Make your changes
4. Test on the robot to ensure it still works
5. Commit with a descriptive message: `git commit -m "Add arm preset positions"`
6. Push: `git push`

### For Experimental Features
If you're working on something experimental that might break things:
1. Create a branch: `git checkout -b feature/new-extending-arm`
2. Make your changes and test
3. Push your branch: `git push -u origin feature/new-extending-arm`
4. Ask a mentor to review before merging or create a pull request using Github.

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

Do not modify files outside of `TeamCode` unless you have a specific reason and you know what you are doing.

## Need Help?

- Ask a mentor or a fellow team member who has experience with software
- Check the [FTC Documentation](https://ftc-docs.firstinspires.org/index.html)
- Visit the [FTC Community Forum](https://ftc-community.firstinspires.org/)