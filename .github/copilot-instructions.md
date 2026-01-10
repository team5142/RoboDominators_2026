# RoboDominators 5142 – Copilot Instructions (WPILib Java)

## Hard requirements (must follow)
- Do not use emojis or non-ASCII characters anywhere in code or comments.
- Target the CURRENT repo state only. Do not assume older WPILib APIs; always look at existing project usage.
- For WPILib / vendor APIs: match the versions in this repo (see build.gradle, gradle.properties, vendordeps/).
- If you are unsure about an API, search within the workspace for an existing usage before inventing one.
- Always ask for access to a file instead of guessing what an unknown class might contain
- Keep comments to a highschool level

## Style
- Keep diffs minimal and compile-safe.
- Prefer small helper methods over repeated logic.
- Avoid per-loop allocations and NT/log spam in periodic loops
- Keep files as short as possible
- Avoid multiline comments whenever possilbe - keep them to 1 or 2 lines max. 
- Avoid comments that only talk about changes made - instead provide permanent comments that describe the current state
- Use the SmartLogger class for logging as a default unless it will cause memory problems

## FRC conventions
- No threads unless explicitly requested.
- Don’t change robot behavior unless asked; performance changes must be behavior-neutral.

## Copilot-specific bad habits to avoid
- Do NOT use "Step 1:", "Step 2:" prefixes in comments - just describe the action directly.
- Verify if code changes complete
- If giving code for copy/paste, clearly define where to paste the block as a whole
- Do NOT repeat unchanged code in diffs - use `// ...existing code...` placeholders instead.
- When showing file changes, always start code blocks with `// filepath: /path/to/file` comment.