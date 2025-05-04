# GitHub Setup Instructions

Follow these steps to create a new GitHub repository for the elliptec-controller package:

1. Create a new repository on GitHub:
   - Go to https://github.com/new
   - Name: `elliptec-controller`
   - Description: `Python controller for Thorlabs Elliptec rotators`
   - Make it Public or Private as you prefer
   - Do not initialize with README, .gitignore, or license files (we already have these)
   - Click "Create repository"

2. Initialize the local Git repository:
   ```bash
   cd /home/maitai/Documents/urashg_2/elliptec-controller
   git init
   git add .
   git commit -m "Initial commit"
   ```

3. Connect to the remote repository (replace 'yourusername' with your GitHub username):
   ```bash
   git remote add origin https://github.com/yourusername/elliptec-controller.git
   git branch -M main
   git push -u origin main
   ```

4. Verify everything is set up correctly:
   ```bash
   git status
   ```

## Publishing to PyPI (Optional)

If you want to make your package installable via pip:

1. Install build tools:
   ```bash
   pip install build twine
   ```

2. Build the package:
   ```bash
   python -m build
   ```

3. Upload to TestPyPI first (for testing):
   ```bash
   python -m twine upload --repository testpypi dist/*
   ```

4. Test the installation:
   ```bash
   pip install --index-url https://test.pypi.org/simple/ elliptec-controller
   ```

5. When ready, upload to the real PyPI:
   ```bash
   python -m twine upload dist/*
   ```

## Development

For developing the package:

```bash
# Install in development mode
pip install -e .

# Run tests
pytest

# Run only the tests that don't require hardware
pytest tests/test_controller.py::TestHexConversion tests/test_controller.py::TestElliptecRotator
```