# AI Agent Guide for "ROS for Mere Mortals"

This document helps AI agents work productively on this book project. It captures the book's purpose, structure, conventions, and maintenance workflows to minimize context overhead across sessions.

## Repository Purpose

This repository contains **"ROS for Mere Mortals" by Michael Wimble** - a practical guide to ROS 2 robotics aimed at hobby roboticists who:
- Don't read ROS source code or research papers regularly
- Struggle with gaps in official ROS 2 documentation
- Need clear explanations of complex concepts, not just quick-reference refreshers
- Want to understand *why* things work, not just *how* to copy-paste commands

**The book's mission**: Do the hard work of reading source code, research papers, and documentation so readers don't have to. Fill gaps in official docs with practical, authoritative explanations.

## Target Audience & Writing Style

### Who Reads This Book
- Hobby roboticists building real robots at home
- People learning ROS 2 from scratch
- Developers frustrated by official documentation that assumes prior knowledge
- Engineers who need deep understanding, not just surface-level tutorials

### Writing Tone
- **Conversational but authoritative**: Like explaining to a friend in a Zoom call
- **Patient and thorough**: Don't assume the reader knows terminology
- **Practical over theoretical**: Real examples from actual robot development
- **Honest about complexity**: "Everything about robots is hard" - acknowledge difficulties
- **Progressive disclosure**: Quick answers for common cases first, then deeper dives

### Style Guidelines
- Use **bold** for important terms and concepts
- Use ***bold italic*** for ROS-specific terms (nodes, topics, packages, etc.)
- Use `code formatting` for commands, file names, and code elements
- Include real code examples that could serve as templates
- Add visual aids (diagrams, screenshots) liberally - "Content without pictures is tiring"
- Use collapsible `<details>` sections for deep dives that casual readers can skip
- Keep chapters digestible - prefer multiple shorter chapters over one massive chapter
- End chapters with "Where to Go From Here" sections to maintain momentum

## Technical Standards

### ROS 2 Versions
- **Primary target**: ROS 2 Jazzy Jalisco
- **Secondary target**: ROS 2 Rolling
- **Do NOT target**: Humble or Iron unless explicitly requested
- Always verify information against actual ROS 2 source code, not just documentation

### Build System
- Use `colcon build --symlink-install` for package builds
- Set `PYTHONNOUSERSITE=1` for colcon builds to avoid stale NumPy paths (see terminal-notes.md in user memory)

### Documentation Philosophy
1. **Source code is truth**: Base explanations on actual ROS 2 repository code
2. **Fill documentation gaps**: Explain things official docs gloss over:
   - Which list items in config files are order-dependent
   - How to debug when things go wrong
   - Parameter interaction and tuning strategies
   - Undocumented command-line options
   - Error messages and their real meanings
3. **Progressive disclosure**: Start simple, build to complex
4. **Practical troubleshooting**: Always include "what to do when it breaks"

## Repository Structure

```
Robotics_Book/
├── AGENTS.md              # This file - AI agent guide
├── ARCHITECTURE.md        # (To be created) Book structure and design
├── README.md              # Project overview
├── LICENSE
├── book/
│   ├── TODO.md           # Living work list (remove completed items, don't check off)
│   ├── chapters/         # Book chapters (markdown)
│   ├── media/            # Images, diagrams, screenshots
│   ├── notes/            # Research notes and drafts
│   └── elements_of_a_urdf.md  # Additional content
├── scripts/              # Build and utility scripts
│   ├── generate_book.sh
│   └── generate_launch_chapter.sh
├── skills/               # May be deleted - see ros2-copilot-skills repo
│   └── copilot-instructions.md
├── custom_behaviors/     # Example ROS 2 packages (working code)
├── description_1/        # Robot description examples
├── description_2/
└── [other ROS 2 packages for examples]
```

### Chapter Organization
- Each chapter is a standalone markdown file in `book/chapters/`
- Chapters use YAML frontmatter for metadata (see Chapter Metadata section)
- Related code examples may be in ROS 2 packages within the repo
- Images go in `book/media/` with descriptive names

## Chapter Metadata Format

Each chapter should have YAML frontmatter:

```yaml
---
title: "Understanding the Workspace Chain"
chapter_number: TBD  # To be assigned during book reorganization
dependencies: ["creating_your_first_workspace_and_package.md"]
related_chapters: ["launch_files.md", "a_bit_about_packages_and_nodes.md"]
related_skills: ["ros2_core/launch_files.md", "ros2_core/parameter_handling.md"]
related_repos: []  # Links to ros2-copilot-skills or wimblerobotics.github.io
last_verified: "2026-05-20"
ros_version: "jazzy"
status: "draft"  # draft | review | complete
---
```

**Purpose of metadata**:
- Track chapter dependencies and reading order
- Enable scripts to validate cross-references
- Record when content was last verified against ROS 2 source
- Help AI agents understand context and relationships
- Support automatic link checking and content synchronization

## Cross-Repository Relationships

### ros2-copilot-skills Repository
- URL: (to be determined)
- Purpose: Reusable AI agent skills for ROS 2 development
- Relationship: This book may reference skills; `/skills/` directory here may be deleted in favor of that repo
- Cross-linking: When relevant, link to detailed skill documentation

### wimblerobotics.github.io Repository  
- URL: https://wimblerobotics.github.io
- Purpose: Author's wiki and knowledge sharing
- Pattern: Uses "where to go next" sections for related content
- Relationship: May cross-link for related topics not covered in book

**Note**: Don't spend effort reading these repos during initial chapter creation. Set up for later integration when we do comprehensive reorganization.

## Content Creation Workflow

### When Creating New Chapters
1. Research the topic in ROS 2 source code first
2. Identify gaps in official documentation
3. Start with common use case (80% of readers)
4. Add progressive detail for advanced users
5. Include troubleshooting section
6. Add "Where to Go From Here" with teasers to related chapters
7. Create chapter metadata frontmatter
8. Add inline code examples (< ~1 page) directly in chapter
9. Put longer examples in separate scripts/files and link to them
10. Include Mermaid diagrams for complex concepts
11. Use `<details>` sections for deep dives
12. Update `book/TODO.md` with any follow-up work identified

### When Creating Example Code
- **Short examples** (< 30 lines): Inline in chapter with syntax highlighting
- **Medium examples** (30-100 lines): Consider inline if essential to flow
- **Long examples** (> 100 lines): Separate file in repo, link from chapter
- All examples must be runnable and tested
- Prefer templates that readers can copy and adapt

### When Creating Diagrams
- Use Mermaid for flowcharts, sequence diagrams, state diagrams
- Renders on GitHub, can export to images for print
- Types to use:
  - `flowchart` for decision trees and process flows
  - `sequenceDiagram` for interaction between components
  - `stateDiagram-v2` for state changes (environment variables, workspace state)
  - `graph` for relationships and dependencies

### Visual Aid Guidelines
- Include diagrams for complex concepts (workspace layering, search order)
- Screenshots for UI interactions (rviz2, terminal output)
- State transition diagrams for environment variable changes
- Before/after comparisons for troubleshooting
- "Pictures greatly add to enjoyment and understandability"

## TODO List Management

**Location**: `book/TODO.md`

**Critical rule**: The TODO is a **living work list**, not a historical document.
- When items are completed: **DELETE THEM** (don't check off as done)
- Keep only current, actionable work items
- Organize by priority and dependencies
- Update automatically as work progresses - don't wait to be reminded

**AI agents should**:
- Update TODO.md when completing work
- Add new items discovered during development
- Remove completed items immediately
- Keep it focused on actionable next steps

## Book Generation & Publishing

### Output Formats
- **Web**: GitHub Pages, potentially GitHub Wiki, markdown with good rendering
- **Print**: Self-published book (via markdown → PDF conversion)
- **Both formats must work**: Test markdown-to-PDF rendering for collapsible sections

### Generation Scripts
- `scripts/generate_book.sh` - Main book generation
- (More scripts to be added for validation, link checking, etc.)

### Future Maintenance Scripts (TODO)
- Validate all internal links
- Check external links
- Verify cross-references to other repos
- Update "last_verified" dates
- Synchronize with ros2-copilot-skills repo
- Export diagrams for print version

## Working with This Repository

### For New Chat Sessions
1. Read this AGENTS.md file first
2. Check `book/TODO.md` for current work priorities
3. Review chapter metadata for relevant chapters
4. Read user memory notes (`terminal-notes.md`)
5. Consult `skills/copilot-instructions.md` for package-specific context

### For Chapter Reviews/Updates
1. Check ROS 2 source code for current accuracy
2. Verify all code examples still work with Jazzy
3. Update `last_verified` date in frontmatter
4. Test that collapsible sections render properly
5. Validate all links (internal and external)

### For Creating Documentation Infrastructure
- **AGENTS.md** (this file): AI agent productivity guide
- **ARCHITECTURE.md**: Book structure, chapter organization, design decisions
- **book/TODO.md**: Current work list (delete completed items)
- **README.md**: Public-facing project description
- **CONTRIBUTING.md**: (If accepting contributions) How to contribute

## Common Patterns & Conventions

### Terminology
- Use ***ROS 2*** (bold italic) when referring to the system
- Use `ros2` (code format) when referring to the command
- Use ***node***, ***topic***, ***package*** (bold italic) for ROS concepts
- Use `package_name`, `node_name` (code format) for specific instances

### File References
- Use relative paths from repo root: `book/chapters/launch_files.md`
- Link to specific line numbers when referencing code: `description_1/urdf/robot.urdf#L42`
- Always use descriptive link text: `[robot URDF](description_1/urdf/robot.urdf)` not `[click here]`

### Command Examples
```bash
# Always include comments explaining what the command does
ros2 launch my_package my_launch.py use_sim_time:=true
```

- Show full commands with realistic arguments
- Don't show every possible option unless in dedicated command reference
- Include expected output when helpful
- Show error messages and how to fix them

### Code Examples
```python
# Include enough context to be runnable
# Use realistic variable names
# Add comments explaining non-obvious parts
def generate_launch_description():
    """
    Brief description of what this launch file does.
    """
    # Implementation...
```

## Questions to Ask Before Writing

When creating new content, verify:
1. **Audience level**: Does this serve beginners, intermediate, or advanced users?
2. **Progressive disclosure**: Can casual readers get a quick answer and skip deep dives?
3. **Source verification**: Is this based on ROS 2 source code or just documentation?
4. **Gap filling**: What do official docs miss that frustrates users?
5. **Troubleshooting**: What goes wrong and how do users fix it?
6. **Visual aids**: Would a diagram make this clearer?
7. **Examples**: Do we need runnable code to demonstrate this?
8. **Cross-references**: What related chapters or skills should readers know about?

## Current Status (Updated 2026-05-20)

### Recently Completed
- Created AGENTS.md (this file)
- Created book/TODO.md structure

### Active Work
- Creating workspace chain chapter series:
  1. Understanding the Workspace Chain: How ROS 2 Finds Your Packages
  2. Managing Multiple Workspaces  
  3. Workspace Troubleshooting Guide

### Known Issues
- All existing chapters need accuracy review against ROS 2 Jazzy source
- Book needs comprehensive reorganization and coherent TOC
- `/skills/` directory may be replaced by ros2-copilot-skills repo
- Need to establish chapter numbering and reading order

### Integration Points
- Cross-reference with ros2-copilot-skills (future)
- Cross-reference with wimblerobotics.github.io (future)
- May publish as GitHub wiki on github.io

---

**Last Updated**: 2026-05-20  
**Maintained By**: AI agents working on this project  
**Update Frequency**: After significant changes to repo structure or conventions
