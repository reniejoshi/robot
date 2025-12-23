## Description
<!--
Summarize the changes made in this PR. 1-2 sentences works great!

Example:
Implemented zeroing logic for collector subsystem.
-->

## Changes Made
<!--
List specific changes in the code. Bullet points are fine!
Think about files, constants, methods, commands, and logging.

Example:
- Added constant for zeroing voltage in `CollectorConstants.java`
- Added method to zero collector pivot motor
- Added command to zero collector when the robot is enabled
- Logged `isZeroed`
-->

## Why These Changes?
<!--
Explain the reasoning behind these changes:
- What problem does this solve?
- Why was this approach chosen?

Example:
These changes ensure that the collector is zeroed before setting its position.
-->

## Testing
<!--
How have you tested these changes?
- Gradle builds without errors?
- Tested in simulation?
- Logging shows expected changes?

Example:
Code builds successfully. Logging shows `isZeroed` is true when the robot is enabled in simulation.
-->

## Breaking Changes
<!--
Are there any changes that might make other code stop working or change how the robot behaves?
- API changes? (Did you change public method names, parameters, or return type?)
- Configuration changes? (Did you change configuration constants or CAN IDs?)
- Behavioral changes? (Does the robot behave differently?)

Examples:
1. No breaking changes.
2. Collector commands will not run until the collector is zeroed.
-->

## Related Issues
<!--
Links to any related issues.

Examples:
- Closes #issue_number
- Related to #issue_number
-->

## Checklist
- [ ] Code follows project coding standards <!--e.g. command-based architecture, naming conventions-->
- [ ] Tests pass locally (`./gradlew test`)
- [ ] Code builds successfully (`./gradlew build`)
- [ ] Documentation <!--comments--> updated if needed
- [ ] No new linting errors <!--errors in style or formatting-->
- [ ] Changes reviewed and approved