#!/bin/bash
# clean_workspace.sh
#
# Safely clean build artifacts from a ROS 2 workspace.
# This is useful when:
#   - You deleted packages and want to remove their build artifacts
#   - Build state is corrupted and you want a fresh start
#   - You changed major dependencies and want to rebuild cleanly
#
# Usage: 
#   cd ~/your_workspace
#   ./clean_workspace.sh [--full]
#
# Options:
#   --full    : Remove build, install, and log directories (full clean)
#   (default) : Interactive - ask which directories to remove

set -e

WORKSPACE_DIR=$(pwd)

echo "========================================="
echo "ROS 2 Workspace Cleanup Script"
echo "========================================="
echo ""
echo "Workspace: $WORKSPACE_DIR"
echo ""

# Check if we're in a ROS 2 workspace
if [ ! -d "src" ]; then
    echo "❌ ERROR: No 'src' directory found in current directory"
    echo "   Are you in a ROS 2 workspace root?"
    echo ""
    echo "   Usage: cd ~/your_workspace && ./clean_workspace.sh"
    exit 1
fi

# Function to show size of a directory
show_size() {
    if [ -d "$1" ]; then
        SIZE=$(du -sh "$1" 2>/dev/null | cut -f1)
        echo "$SIZE"
    else
        echo "N/A"
    fi
}

# Show what exists
echo "Current workspace contents:"
echo "  src/     : $(show_size src)"
echo "  build/   : $(show_size build)"
echo "  install/ : $(show_size install)"
echo "  log/     : $(show_size log)"
echo ""

# Full clean mode
if [ "$1" == "--full" ]; then
    echo "Full clean mode: removing build/, install/, and log/"
    echo ""
    read -p "This will delete all build artifacts. Continue? (y/N) " -n 1 -r
    echo ""
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        echo "Removing build/..."
        rm -rf build
        echo "Removing install/..."
        rm -rf install
        echo "Removing log/..."
        rm -rf log
        echo ""
        echo "✓ Workspace cleaned successfully"
        echo ""
        echo "Next steps:"
        echo "  1. colcon build --symlink-install"
        echo "  2. source install/setup.bash"
    else
        echo "Cancelled."
        exit 0
    fi
    exit 0
fi

# Interactive mode
echo "What would you like to clean?"
echo ""
echo "1) build/   - Build artifacts (safe to delete, will rebuild)"
echo "2) install/ - Installed packages (safe to delete, will rebuild)"  
echo "3) log/     - Build logs (safe to delete, just logs)"
echo "4) All of the above (full clean)"
echo "5) Specific package only"
echo "6) Cancel"
echo ""
read -p "Choose option (1-6): " -n 1 -r
echo ""
echo ""

case $REPLY in
    1)
        echo "Removing build/..."
        rm -rf build
        echo "✓ Done"
        ;;
    2)
        echo "Removing install/..."
        rm -rf install
        echo "✓ Done"
        echo ""
        echo "⚠ Remember to re-source your workspace after rebuilding"
        ;;
    3)
        echo "Removing log/..."
        rm -rf log
        echo "✓ Done"
        ;;
    4)
        echo "Removing build/, install/, and log/..."
        rm -rf build install log
        echo "✓ Done"
        echo ""
        echo "Next steps:"
        echo "  1. colcon build --symlink-install"
        echo "  2. source install/setup.bash"
        ;;
    5)
        echo "Enter package name to clean:"
        read PACKAGE_NAME
        if [ -z "$PACKAGE_NAME" ]; then
            echo "❌ No package name provided"
            exit 1
        fi
        echo ""
        echo "Cleaning package: $PACKAGE_NAME"
        
        # Check if package exists in src
        if [ ! -d "src/$PACKAGE_NAME" ]; then
            echo "⚠ WARNING: Package not found in src/$PACKAGE_NAME"
            echo "   Continuing anyway to clean build artifacts..."
        fi
        
        # Remove from build and install
        if [ -d "build/$PACKAGE_NAME" ]; then
            echo "  Removing build/$PACKAGE_NAME"
            rm -rf "build/$PACKAGE_NAME"
        fi
        if [ -d "install/$PACKAGE_NAME" ]; then
            echo "  Removing install/$PACKAGE_NAME"
            rm -rf "install/$PACKAGE_NAME"
        fi
        
        # Clean from install/lib if it's a Python package
        if [ -d "install/lib/python3.10/site-packages/$PACKAGE_NAME" ]; then
            echo "  Removing Python package from install/"
            rm -rf "install/lib/python3.10/site-packages/$PACKAGE_NAME"
        fi
        
        echo "✓ Done"
        echo ""
        echo "Next steps:"
        echo "  1. If you deleted the package: rm -rf src/$PACKAGE_NAME"
        echo "  2. If you're rebuilding it: colcon build --packages-select $PACKAGE_NAME"
        ;;
    6)
        echo "Cancelled."
        exit 0
        ;;
    *)
        echo "❌ Invalid option"
        exit 1
        ;;
esac

echo ""
echo "========================================="
echo "Cleanup Complete"
echo "========================================="
echo ""

# Show new sizes
if [ -d "build" ] || [ -d "install" ] || [ -d "log" ]; then
    echo "Remaining workspace contents:"
    if [ -d "build" ]; then echo "  build/   : $(show_size build)"; fi
    if [ -d "install" ]; then echo "  install/ : $(show_size install)"; fi
    if [ -d "log" ]; then echo "  log/     : $(show_size log)"; fi
else
    echo "All build artifacts removed."
    echo ""
    echo "Remember to rebuild:"
    echo "  colcon build --symlink-install"
fi
echo ""
