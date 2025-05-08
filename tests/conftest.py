"""
Configuration for pytest.

This file contains configuration and fixtures for pytest when running tests 
for the elliptec_controller package.
"""

import os
import sys

# Make the package available for imports during testing
package_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
if package_root not in sys.path:
    sys.path.insert(0, package_root)