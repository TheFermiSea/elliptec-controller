# Contributing to elliptec-controller

Thank you for considering contributing to the elliptec-controller package! This document provides guidelines and instructions for contributing.

## Code of Conduct

Please be respectful and considerate of others when contributing to this project. We aim to foster an inclusive and welcoming community.

## How to Contribute

There are many ways to contribute to this project:

1. **Reporting bugs**: If you find a bug, please create an issue describing the problem, including steps to reproduce it.

2. **Suggesting features**: If you have ideas for new features or improvements, feel free to create an issue describing your suggestion.

3. **Contributing code**: We welcome pull requests for bug fixes, features, or improvements.

4. **Improving documentation**: Documentation improvements are always appreciated.

## Development Setup

1. Fork the repository on GitHub.

2. Clone your fork locally:
   ```bash
   git clone https://github.com/your-username/elliptec-controller.git
   cd elliptec-controller
   ```

3. Create a virtual environment and install the package in development mode:
   ```bash
   python -m venv venv
   source venv/bin/activate  # On Windows: venv\Scripts\activate
   pip install -e .
   pip install -r requirements.txt
   ```

4. Make your changes.

5. Run tests to ensure everything is working:
   ```bash
   pytest tests/test_controller.py::TestHexConversion tests/test_controller.py::TestElliptecRotator
   ```
   Note: Some tests require hardware and might be skipped.

6. Push your changes to your fork and submit a pull request.

## Pull Request Guidelines

1. Update documentation if needed.
2. Add or update tests for your changes.
3. Ensure your code follows the project's coding style.
4. Make sure all tests pass.
5. Keep your pull request focused on a single topic.

## Hardware Testing

If your changes involve hardware interaction, please test with real hardware if possible. If not, please note this in your pull request.

## Code Style

We follow PEP 8 coding standards. You can use tools like `black` and `flake8` to format and check your code:

```bash
pip install black flake8
black elliptec_controller/
flake8 elliptec_controller/
```

## License

By contributing to this project, you agree that your contributions will be licensed under the project's MIT License.

Thank you for your contributions!