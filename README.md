# Infinite-Recharge-2551
FRC Team 2551 Penguin Empire Robotics' code for the 2020 season, Infinite Recharge.

## Software features

Hopefully lots. :smile:

## Useful links - reference, tips, resources, etc.

This list in intended for internal consumption on Team 2551. Links may go to the C++-specific versions of things, and it's written in an informal style. If you find it useful regardless, fantastic, but 

This is an incomplete list - more coming soon.

### General FRC stuff

- **http://docs.wpilib.org**. This is a very useful website.
- Also useful are the WPILibC (they call it that, but it's the C++ version) API docs, at https://first.wpi.edu/FRC/roborio/release/docs/cpp/index.html.

### Phoenix/CTRE/Talons

- The software for your computer can be downloaded at http://www.ctr-electronics.com/control-system/hro.html#product_tabs_technical_resources. You want the 'CTRE Phoenix Framework Installer'. Not sure if this will install on Mac.
- Software documentation for CTRE stuff is at https://phoenix-documentation.readthedocs.io/en/latest/index.html
- API docs for C++ version of `CTRE_Phoenix` are at http://www.ctr-electronics.com/downloads/api/cpp/html/index.html. Note that (per [CTRE's 2020 Kickoff Blog](https://phoenix-documentation.readthedocs.io/en/latest/blog/blog-kickoff-2020.html#online-api-documentation)) **this is not yet up-to-date with the 2020 version of the library** (at least as of late on 2020-01-04).
- Examples of the API in use can be found at https://github.com/CrossTheRoadElec/Phoenix-Examples-Languages/tree/master/C%2B%2B

### REV stuff (NEO and SPARK MAX)

- Spark Max software resources: http://www.revrobotics.com/sparkmax-software/#cpp-api. Includes download of the client application for the MAXs (which you need), firmware downloads, information about the API (including [online](http://www.revrobotics.com/content/sw/max/sw-docs/cpp/index.html) and offline versions of the reference docs), code examples, etc.
- Spark Max User Manual: http://www.revrobotics.com/sparkmax-users-manual/

### Kauai Labs NavX

- https://pdocs.kauailabs.com/navx-mxp/software/roborio-libraries/c/
  - Software install is linked at "latest build" in the third paragraph. This installs "RoboRIO Libraries and Examples, the navXUI and the navX Tools"; see the readme in the downloaded zip for more information.
  - The example are also available online [here](https://pdocs.kauailabs.com/navx-mxp/examples/).
  - API docs are available online at https://www.kauailabs.com/public_files/navx-mxp/apidocs/c++/.

### CI/Azure Pipelines Resources

These are not things you have to download or that you're going to be looking at every day, like most of the other subheadings, but it's useful stuff to have as institutional knowledge, I think, so it needs to go somewhere.

- WPILib's roboRIO docker images: https://hub.docker.com/r/wpilib/roborio-cross-ubuntu/tags
- Not sure which, if any, of these are important:
  - [Announcing Azure Pipelines with unlimited CI/CD minutes for open source | Microsoft Azure Blog](https://azure.microsoft.com/en-us/blog/announcing-azure-pipelines-with-unlimited-ci-cd-minutes-for-open-source/)
  - [Tutorial: Create a CI/CD pipeline for your existing code by using Azure DevOps Projects | Azure Docs](https://docs.microsoft.com/en-us/azure/devops-project/azure-devops-project-github)
  - [Azure Pipelines documentation](https://docs.microsoft.com/en-us/azure/devops/pipelines/?view=azure-devops) seems useful. Specifically, looked at:
    - [Build GitHub repositories](https://docs.microsoft.com/en-us/azure/devops/pipelines/repos/github?tabs=yaml&view=azure-devops)
    - [Create your first pipeline](https://docs.microsoft.com/en-us/azure/devops/pipelines/create-first-pipeline?view=azure-devops&tabs=browser%2Ctfs-2018-2)
  - [Defining the mergeability of pull requests | GitHub Help](https://help.github.com/en/github/administering-a-repository/defining-the-mergeability-of-pull-requests)

### Limelight Resources

- Docs: http://docs.limelightvision.io/en/latest/
- Downloads: https://limelightvision.io/pages/downloads (nothing here is necessary for day-to-day usage of the LL, I think. It's all tools for flashing it (which should happen at least once a year, and possibly whenever a new image is released).

### Other stuff

- _Controls Engineering in the FIRST Robotics Competition: Graduate-level control theory for high schoolers_ by Tyler Veness (a 3512 mentor) is an entire book about ways to model (robot) systems and write better code for them, and the latest version is always available at https://file.tavsys.net/control/controls-engineering-in-frc.pdf. (The introduction explains it better.)
