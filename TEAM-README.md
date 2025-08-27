# FTC Team 18650 - The Cookie Clickers
## Bennington, VT

This is Team 18650's fork of the official FTC SDK. Our TeamCode was ported from our 2024-2025 INTO THE DEEP season and adapted for future seasons. For SDK documentation, see [README.md](README.md).

## Documentation Structure

We maintain our team documentation separately from the FTC SDK to preserve upstream compatibility:

- **README.md** - Official FTC SDK documentation (DO NOT MODIFY)
- **TEAM-README.md** - This file, our main team documentation
- **GETTING-STARTED.md** - Onboarding guide for new team members
- **CLAUDE.md** - AI assistant context for Claude Code
- **TeamCode/...teamcode/readme.md** - TeamCode-specific documentation
- **docs/** - Additional detailed documentation

## Quick Links

- [Getting Started](GETTING-STARTED.md) - New member onboarding
- [TeamCode Documentation](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/readme.md) - Code architecture
- [Official FTC Docs](https://ftc-docs.firstinspires.org/) - Competition resources

## Repository Management

### Syncing with Upstream FTC SDK
```bash
# Add upstream remote (one time)
git remote add upstream https://github.com/FIRST-Tech-Challenge/FtcRobotController.git

# Sync with upstream
git fetch upstream
git merge upstream/master
```

When syncing, our custom documentation files won't conflict since they don't exist in the upstream repository.