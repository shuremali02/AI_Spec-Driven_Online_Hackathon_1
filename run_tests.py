#!/usr/bin/env python3
"""
Test runner for RAG Chatbot backend tests
"""
import subprocess
import sys
import os


def run_tests():
    """Run all tests using pytest"""
    # Change to backend directory
    os.chdir(os.path.join(os.path.dirname(__file__), "backend"))

    # Run pytest with verbose output
    result = subprocess.run([
        sys.executable, "-m", "pytest",
        "tests/",
        "-v",
        "--tb=short",
        "-x"  # Stop on first failure
    ])

    return result.returncode


if __name__ == "__main__":
    exit_code = run_tests()
    sys.exit(exit_code)