# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

- CICD: Add coverage, lint and test workflows
- Docstrings for every public module, class, method and function
- Support custom velocity limits for robot descriptions with continuous joints
- Support pixi for development and running examples
- examples: Record videos for all or some scenarios in the library

### Changed

- Bump minimum Pink version to 4.4.0
- Bump minimum Python version to 3.11
- Scenarios: update the 7-dof Kinova Gen3 description
- Scenarios: update the Universal Robots descriptions
- Transfer copyright notices to `NOTICE` file
- examples: Rename scenario parg to --robot kwarg

### Removed

- Remove `environment.yaml` as we are now using pixi
- Remove `RunningMeanStd` as it was unused

## [0.2.0] - 2024-12-16

Still a work in progress. Changelog will start from next version.

## [0.1.0] - 2024-12-13

Work-in-progress version of the project.

[unreleased]: https://github.com/stephane-caron/pink_bench/compare/v0.2.0...HEAD
[0.2.0]: https://github.com/stephane-caron/pink_bench/compare/v0.1.0...v0.2.0
[0.1.0]: https://github.com/stephane-caron/pink_bench/releases/tag/v0.1.0
