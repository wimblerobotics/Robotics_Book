#!/bin/bash
# check_workspace_environment.sh
# 
# Diagnostic script to show your current ROS 2 workspace environment state.
# Run this when you're confused about which packages will be found.
#
# Usage: ./check_workspace_environment.sh [package_name]

set -e

echo "======================================"
echo "ROS 2 Workspace Environment Diagnostic"
echo "======================================"
echo ""

# Check if ROS 2 is sourced at all
if [ -z "$AMENT_PREFIX_PATH" ]; then
    echo "❌ ERROR: No ROS 2 environment sourced!"
    echo "   You need to source a workspace or /opt/ros/jazzy/setup.bash"
    echo ""
    echo "   Try: source /opt/ros/jazzy/setup.bash"
    echo "   Or:  source ~/ros2_ws/install/setup.bash"
    exit 1
fi

echo "✓ ROS 2 environment is sourced"
echo ""

# Show the workspace search path
echo "Workspace Search Path (in order):"
echo "----------------------------------"
echo "$AMENT_PREFIX_PATH" | tr ':' '\n' | nl -w2 -s'. '
echo ""

# Show which workspace is searched first (highest priority)
FIRST_WORKSPACE=$(echo "$AMENT_PREFIX_PATH" | cut -d':' -f1)
echo "Highest Priority Workspace: $FIRST_WORKSPACE"
echo ""

# If a package name was provided, show where it will be found
if [ -n "$1" ]; then
    PACKAGE_NAME="$1"
    echo "Package Search Results for: $PACKAGE_NAME"
    echo "----------------------------------------"
    
    # Check if package exists at all
    if ros2 pkg list | grep -q "^${PACKAGE_NAME}$"; then
        echo "✓ Package found"
        echo ""
        
        # Show which location will be used
        USED_LOCATION=$(ros2 pkg prefix "$PACKAGE_NAME" 2>/dev/null)
        echo "Will use package from:"
        echo "  $USED_LOCATION"
        echo ""
        
        # Show all locations where this package exists
        echo "All locations of this package:"
        ros2 pkg prefix --all "$PACKAGE_NAME" 2>/dev/null | nl -w2 -s'. '
        echo ""
        
        # Check if there are multiple versions
        NUM_LOCATIONS=$(ros2 pkg prefix --all "$PACKAGE_NAME" 2>/dev/null | wc -l)
        if [ "$NUM_LOCATIONS" -gt 1 ]; then
            echo "⚠ WARNING: Package exists in $NUM_LOCATIONS locations!"
            echo "   The first one will be used (highest priority)"
            echo "   Others will be ignored"
        fi
    else
        echo "❌ Package '$PACKAGE_NAME' NOT FOUND in any workspace"
        echo ""
        echo "   Troubleshooting:"
        echo "   1. Is the package built? Check: ls ~/ros2_ws/install/"
        echo "   2. Did you source the workspace containing it?"
        echo "   3. Check spelling: ros2 pkg list | grep -i '$PACKAGE_NAME'"
    fi
    echo ""
fi

# Show other important environment variables
echo "Other Important Variables:"
echo "--------------------------"
echo "CMAKE_PREFIX_PATH entries: $(echo $CMAKE_PREFIX_PATH | tr ':' '\n' | wc -l)"
echo "PYTHONPATH entries: $(echo $PYTHONPATH | tr ':' '\n' | wc -l)"
echo "LD_LIBRARY_PATH entries: $(echo $LD_LIBRARY_PATH | tr ':' '\n' | wc -l)"
echo ""

# Check for common problems
echo "Problem Detection:"
echo "------------------"

# Check for duplicates in AMENT_PREFIX_PATH
DUPLICATES=$(echo "$AMENT_PREFIX_PATH" | tr ':' '\n' | sort | uniq -d)
if [ -n "$DUPLICATES" ]; then
    echo "⚠ WARNING: Duplicate paths detected in AMENT_PREFIX_PATH:"
    echo "$DUPLICATES" | nl -w2 -s'. '
    echo "   This usually means you sourced multiple times in this terminal."
    echo "   Recommendation: Open a fresh terminal and source only once."
    echo ""
else
    echo "✓ No duplicate paths in AMENT_PREFIX_PATH"
    echo ""
fi

# Check if /opt/ros is in the path but not last
if echo "$AMENT_PREFIX_PATH" | grep -q "/opt/ros"; then
    LAST_PATH=$(echo "$AMENT_PREFIX_PATH" | tr ':' '\n' | tail -1)
    if ! echo "$LAST_PATH" | grep -q "/opt/ros"; then
        echo "⚠ WARNING: /opt/ros is not the last workspace in search path"
        echo "   This might indicate incorrect sourcing order"
        echo "   Typically /opt/ros/jazzy should be the last (lowest priority)"
        echo ""
    fi
fi

# Check if multiple workspaces from same base exist
WS_COUNT=$(echo "$AMENT_PREFIX_PATH" | tr ':' '\n' | grep -c "/install$" || true)
if [ "$WS_COUNT" -gt 2 ]; then
    echo "⚠ INFO: You have $WS_COUNT custom workspaces in the chain"
    echo "   This is fine, but make sure you intended this"
    echo "   Each workspace adds complexity to package resolution"
    echo ""
fi

echo "======================================"
echo "Diagnostic Complete"
echo "======================================"
echo ""
echo "Quick Tips:"
echo "  - Your workspace is searched in the order shown above"
echo "  - First match wins when looking for packages"
echo "  - If wrong package loads, check the search order"
echo "  - To reset: open new terminal and source correctly"
echo ""
echo "For more help, see: workspace_troubleshooting.md"
