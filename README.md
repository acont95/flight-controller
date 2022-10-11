<!-- Improved compatibility of back to top link: See: https://github.com/othneildrew/Best-README-Template/pull/73 -->
<a name="readme-top"></a>
<!--
*** Thanks for checking out the Best-README-Template. If you have a suggestion
*** that would make this better, please fork the repo and create a pull request
*** or simply open an issue with the tag "enhancement".
*** Don't forget to give the project a star!
*** Thanks again! Now go create something AMAZING! :D
-->



<!-- PROJECT SHIELDS -->
<!--
*** I'm using markdown "reference style" links for readability.
*** Reference links are enclosed in brackets [ ] instead of parentheses ( ).
*** See the bottom of this document for the declaration of the reference variables
*** for contributors-url, forks-url, etc. This is an optional, concise syntax you may use.
*** https://www.markdownguide.org/basic-syntax/#reference-style-links
-->
[![Contributors][contributors-shield]][contributors-url]
[![Forks][forks-shield]][forks-url]
[![Stargazers][stars-shield]][stars-url]
[![Issues][issues-shield]][issues-url]
[![MIT License][license-shield]][license-url]
[![LinkedIn][linkedin-shield]][linkedin-url]



<!-- PROJECT LOGO -->
<br />
<div align="center">
  <a href="https://github.com/acont95/flight-controller">
    <img src="images/air-drone-icon.png" alt="Logo" width="80">
  </a>

<h3 align="center">Quadcopter Flight Controller</h3>

  <p align="center">
    A simple flight controller running on MbedOS.
    <br />
    <a href="https://github.com/acont95/flight-controller"><strong>Explore the docs »</strong></a>
    <br />
    <br />
    <a href="https://github.com/acont95/flight-controller">View Demo</a>
    ·
    <a href="https://github.com/acont95/flight-controller/issues">Report Bug</a>
    ·
    <a href="https://github.com/acont95/flight-controller/issues">Request Feature</a>
  </p>
</div>



<!-- TABLE OF CONTENTS -->
<details>
  <summary>Table of Contents</summary>
  <ol>
    <li>
      <a href="#about-the-project">About The Project</a>
      <ul>
        <li><a href="#built-with">Built With</a></li>
      </ul>
    </li>
    <li>
      <a href="#getting-started">Getting Started</a>
      <ul>
        <li><a href="#prerequisites">Prerequisites</a></li>
        <li><a href="#installation">Installation</a></li>
      </ul>
    </li>
    <li><a href="#usage">Usage</a></li>
    <li><a href="#roadmap">Roadmap</a></li>
    <li><a href="#contributing">Contributing</a></li>
    <li><a href="#license">License</a></li>
    <li><a href="#contact">Contact</a></li>
    <li><a href="#acknowledgments">Acknowledgments</a></li>
  </ol>
</details>



<!-- ABOUT THE PROJECT -->
## About The Project

<!-- ![Product Name Screen Shot][product-screenshot] -->

This goal of this project is to develop a simple yet fully functional flight controller for quadcopter UAV's to showcase all the components needed to develop such a system from scratch. Any MCU which can run MbedOS should be compatible. MbedOS specific drivers are included for an ICM20948 IMU, HCSR04 ultrasonic sensor, MS5611 barometer/thermometer. Other sensors may be supported in the future. An single board computer is used for transceiving video, flight data, and flight commands over WiFi to a ground control station. A Qt based application is included to communicate with the drone.

<p align="right">(<a href="#readme-top">back to top</a>)</p>



### Built With

* [![C++][C++]][Cpp-url]
* [![Python][Python]][Python-url]
* [![PlatformIO][PlatformIO]][PlatformIO-url]
* [![Qt][Qt]][Qt-url]


<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- GETTING STARTED -->
## Getting Started

This is an example of how you may give instructions on setting up your project locally.
To get a local copy up and running follow these simple example steps.

### Prerequisites

This is an example of how to list things you need to use the software and how to install them.
* npm
  ```sh
  npm install npm@latest -g
  ```

### Installation

1. Get a free API Key at [https://example.com](https://example.com)
2. Clone the repo
   ```sh
   git clone https://github.com/acont95/flight-controller.git
   ```
3. Install NPM packages
   ```sh
   npm install
   ```
4. Enter your API in `config.js`
   ```js
   const API_KEY = 'ENTER YOUR API';
   ```

<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- USAGE EXAMPLES -->
## Usage

Use this space to show useful examples of how a project can be used. Additional screenshots, code examples and demos work well in this space. You may also link to more resources.

_For more examples, please refer to the [Documentation](https://example.com)_

<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- ROADMAP -->
## Roadmap

- [ ] Feature 1
- [ ] Feature 2
- [ ] Feature 3
    - [ ] Nested Feature

See the [open issues](https://github.com/acont95/flight-controller/issues) for a full list of proposed features (and known issues).

<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- CONTRIBUTING -->
## Contributing

Contributions are what make the open source community such an amazing place to learn, inspire, and create. Any contributions you make are **greatly appreciated**.

If you have a suggestion that would make this better, please fork the repo and create a pull request. You can also simply open an issue with the tag "enhancement".
Don't forget to give the project a star! Thanks again!

1. Fork the Project
2. Create your Feature Branch (`git checkout -b feature/AmazingFeature`)
3. Commit your Changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the Branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- LICENSE -->
## License

Distributed under the MIT License. See `LICENSE.txt` for more information.

<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- CONTACT -->
## Contact

Alex Conticello - conticello.alex@gmail.com

Project Link: [https://github.com/acont95/flight-controller](https://github.com/acont95/flight-controller)

<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- ACKNOWLEDGMENTS -->
## Acknowledgments

* []()
* []()
* []()

<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- MARKDOWN LINKS & IMAGES -->
<!-- https://www.markdownguide.org/basic-syntax/#reference-style-links -->
[contributors-shield]: https://img.shields.io/github/contributors/acont95/flight-controller.svg?style=for-the-badge
[contributors-url]: https://github.com/acont95/flight-controller/graphs/contributors
[forks-shield]: https://img.shields.io/github/forks/acont95/flight-controller.svg?style=for-the-badge
[forks-url]: https://github.com/acont95/flight-controller/network/members
[stars-shield]: https://img.shields.io/github/stars/acont95/flight-controller.svg?style=for-the-badge
[stars-url]: https://github.com/acont95/flight-controller/stargazers
[issues-shield]: https://img.shields.io/github/issues/acont95/flight-controller.svg?style=for-the-badge
[issues-url]: https://github.com/acont95/flight-controller/issues
[license-shield]: https://img.shields.io/github/license/acont95/flight-controller.svg?style=for-the-badge
[license-url]: https://github.com/acont95/flight-controller/blob/master/LICENSE.txt
[linkedin-shield]: https://img.shields.io/badge/-LinkedIn-black.svg?style=for-the-badge&logo=linkedin&colorB=555
[linkedin-url]: https://linkedin.com/in/alex-conticello-8555bb101
[product-screenshot]: images/air-drone-icon.png

[C++]: https://img.shields.io/badge/-c++-black?style=for-the-badge&logo=c%2B%2B&logoColor=61DAFB
[Cpp-url]: https://isocpp.org/

[PlatformIO]: https://img.shields.io/badge/-PlatformIO-blue?style=for-the-badge&logo=Python
[PlatformIO-url]: https://platformio.org/

[Python]: https://img.shields.io/badge/-Python-green?style=for-the-badge&logo=Python
[Python-url]: https://www.python.org/

[Qt]: https://img.shields.io/badge/-qt-grey.svg?style=for-the-badge&logo=qt
[Qt-url]: https://www.qt.io/