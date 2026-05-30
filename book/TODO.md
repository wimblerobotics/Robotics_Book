# TODO - ROS for Mere Mortals Book

**IMPORTANT**: This is a living work list. When items are completed, DELETE them (don't check them off). Only current, actionable work should remain.

---

## Immediate Priorities

### Workspace Chain Chapter Series
- [ ] Test collapsible `<details>` sections in markdown-to-PDF conversion
- [ ] Verify all Mermaid diagrams render in both web and print formats
- [ ] Create additional example script: compare_workspace_package.py (show differences between package versions)
- [ ] Test all example scripts with real ROS 2 Jazzy installation

### Book Architecture & Organization

#### Critical Restructuring
- [ ] Design comprehensive Table of Contents with logical chapter progression
- [ ] Create chapter outline for each proposed chapter (1-2 paragraphs per chapter)
- [ ] Establish chapter numbering scheme and reading dependencies
- [ ] Define topic segregation strategy (which content goes in which chapter)
- [ ] Map out "Where to Go From Here" links across all chapters
- [ ] Create decision tree: which chapters must be read sequentially vs. standalone

#### Documentation Infrastructure
- [ ] Create ARCHITECTURE.md documenting book structure and design decisions
- [ ] Create README.md for public-facing project description
- [ ] Consider CONTRIBUTING.md if accepting external contributions
- [ ] Design metadata validation scripts (verify frontmatter consistency)
- [ ] Create link validation script (check internal and external links)
- [ ] Create cross-reference validation script (verify chapter dependencies exist)

### Existing Chapter Review & Rewrite

#### Accuracy Verification
- [ ] Review launch_files.md for accuracy against ROS 2 Jazzy source code
- [ ] Review launch2.md - determine if it should merge with launch_files.md
- [ ] Review creating_your_first_workspace_and_package.md against Jazzy
- [ ] Review a_bit_about_packages_and_nodes.md against Jazzy
- [ ] Review navigating_with_rviz.md against Jazzy and Nav2 current behavior
- [ ] Review all other existing chapters for technical accuracy

#### Content Integration
- [ ] Identify overlapping content across chapters that needs consolidation
- [ ] Identify gaps where new chapters are needed
- [ ] Ensure consistent terminology usage across all chapters
- [ ] Standardize code example format and style
- [ ] Add chapter metadata frontmatter to all existing chapters
- [ ] Add "Where to Go From Here" sections to chapters that lack them

### Visual Content

#### Diagrams & Images
- [ ] Create troubleshooting decision trees
- [ ] Add screenshots for common error messages
- [ ] Create before/after comparisons for workspace fixes
- [ ] Review all existing diagrams for accuracy and clarity

#### Image Organization
- [ ] Establish naming convention for media files
- [ ] Organize media/ directory by chapter or topic
- [ ] Create alt-text guidelines for accessibility
- [ ] Test image rendering in both web and print formats

### Example Code & Scripts

#### Workspace Management Examples
- [ ] Create `examples/workspace_chain/demo_workspace_order.sh` - demonstrate search behavior
- [ ] Create `examples/workspace_chain/test_package_priority.py` - show which package loads
- [ ] Create `examples/workspace_chain/cleanup_workspace.sh` - safe cleanup procedures
- [ ] Create `examples/workspace_chain/check_environment.sh` - debug environment variables
- [ ] Document when to inline examples vs. link to separate files

#### Other Example Needs
- [ ] Identify which chapters need runnable example code
- [ ] Create template launch files for common scenarios
- [ ] Create template package structures
- [ ] Ensure all examples are tested with ROS 2 Jazzy

### Cross-Repository Integration (Future)

#### ros2-copilot-skills Integration
- [ ] Decide whether to keep or delete local `/skills/` directory
- [ ] Map book chapters to relevant skills in ros2-copilot-skills repo
- [ ] Add cross-references in chapter metadata
- [ ] Create script to validate skill links
- [ ] Consider: should skills be embedded in chapters or just linked?

#### wimblerobotics.github.io Integration
- [ ] Review "where to go next" pattern from wiki
- [ ] Identify topics covered in wiki that should be in book
- [ ] Identify topics in book that should cross-reference wiki
- [ ] Add relevant links to chapter metadata
- [ ] Ensure no content duplication without purpose

### Publication & Build System

#### Multi-Format Output
- [ ] Test current markdown-to-PDF conversion with collapsible sections
- [ ] Evaluate whether `<details>` sections work acceptably in print
- [ ] If not: design alternative "Deep Dive" section formatting
- [ ] Create print-specific CSS/styling for PDF generation
- [ ] Test Mermaid diagram export for print quality
- [ ] Design page layout for print version (margins, headers, footers)

#### Web Publication
- [ ] Decide: GitHub wiki vs. GitHub Pages vs. both?
- [ ] Design navigation structure for web version
- [ ] Create "Copy Markdown" buttons for AI agent consumption
- [ ] Test rendering on GitHub's markdown renderer
- [ ] Ensure mobile-friendly formatting

#### Build Scripts
- [ ] Document usage of existing `scripts/generate_book.sh`
- [ ] Create script to validate all chapters before build
- [ ] Create script to auto-generate Table of Contents
- [ ] Create script to check for broken links
- [ ] Create script to update "last_verified" dates
- [ ] Create script to validate chapter metadata consistency

### Content Deep Dives (Future Chapters)

#### Topics Needing Full Chapters
- [ ] ros2_control comprehensive guide
- [ ] Parameter management and YAML configuration deep dive
- [ ] Nav2 costmap configuration and tuning (explain layer ordering, critique interaction)
- [ ] Behavior trees for mere mortals
- [ ] URDF and robot description
- [ ] Debugging ROS 2 systems (processes, tools, strategies)
- [ ] TF2 and coordinate frames explained
- [ ] Custom message, service, and action definitions

#### Behind-the-Scenes Supplements
- [ ] How the `ros2` command works internally
- [ ] How `colcon build` processes packages
- [ ] How `source` modifies environment
- [ ] How ROS 2 searches for packages, nodes, plugins
- [ ] How parameter loading actually works

#### Command Reference Supplements
- [ ] `ros2 launch` comprehensive reference
- [ ] `ros2 run` comprehensive reference  
- [ ] `ros2 param` comprehensive reference
- [ ] `colcon` command reference
- [ ] Debugging command cheat sheet

### Writing Style & Pattern Refinement

#### "Cliff Hanger" Link Pattern
- [ ] Develop formula for "Where to Go From Here" sections
- [ ] Create engaging teasers that make readers want to continue
- [ ] Balance between providing complete info vs. encouraging exploration
- [ ] Test with sample readers for effectiveness

#### Progressive Disclosure
- [ ] Define levels: Quick Answer → Common Case → Advanced → Deep Dive
- [ ] Create visual markers for each level
- [ ] Ensure readers know when to stop if they just need basics
- [ ] Test collapsible sections with target audience

#### Troubleshooting Pattern
- [ ] Standardize troubleshooting section format across chapters
- [ ] Symptom → Diagnosis → Solution structure
- [ ] Include actual error messages with fixes
- [ ] Add "How to debug this yourself" guidance

### Quality Assurance

#### Technical Review
- [ ] Verify all content against ROS 2 Jazzy source code (not just docs)
- [ ] Test all code examples on actual ROS 2 Jazzy installation
- [ ] Verify all command examples produce expected output
- [ ] Check for deprecated commands or patterns
- [ ] Ensure version-specific content is clearly marked

#### Readability Review
- [ ] Check chapter lengths - are any too long/intimidating?
- [ ] Ensure consistent voice and tone
- [ ] Remove jargon or define it on first use
- [ ] Verify all links work (internal and external)
- [ ] Check for typos and grammar

#### Accessibility
- [ ] Add alt-text to all images
- [ ] Ensure diagrams have text descriptions
- [ ] Check heading hierarchy (proper H1, H2, H3 nesting)
- [ ] Test with screen readers if targeting web format

---

## Meta TODO Items

These are about maintaining the TODO itself:

- [ ] Periodically review and prune outdated items
- [ ] Group related items for batch completion
- [ ] Move completed items to archive (separate doc) if historical record needed
- [ ] Keep priorities clearly marked
- [ ] Update this file automatically after completing any work

---

## Recently Completed (2026-05-20)

The following chapters and infrastructure were just created and are ready for review:

**New Chapters** (in `book/chapters/`):
- `understanding_the_workspace_chain.md` - Core concepts of how ROS 2 finds packages
- `managing_multiple_workspaces.md` - Advanced multi-workspace management
- `workspace_troubleshooting.md` - Diagnostic guide for common problems

**New Scripts** (in `scripts/`):
- `check_workspace_environment.sh` - Diagnostic tool showing current workspace state
- `clean_workspace.sh` - Safe workspace cleanup utility

**New Infrastructure**:
- `AGENTS.md` - AI agent guide for working on this book project
- `book/TODO.md` - This file, living work list

**Next Review Steps**:
1. Read through the three workspace chapters for technical accuracy
2. Test example scripts on actual ROS 2 Jazzy system
3. Check if collapsible sections render acceptably in markdown-to-PDF
4. Verify Mermaid diagrams display correctly
5. Decide on overall book structure and chapter ordering

---

**Last Updated**: 2026-05-20  
**Principle**: Delete completed items; keep only actionable work
