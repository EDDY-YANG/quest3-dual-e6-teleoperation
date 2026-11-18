# Acknowledgments and references

We thank the authors of RangedIK and its open-source implementation, and the researchers who established damped least-squares inverse kinematics. Their work provides the motion-generation and numerical foundations used in this project's development. The Quest/E6 interfaces, calibration workflow and integration are described separately from these upstream contributions.

The correct abbreviation for **damped least squares is DLS**, rather than DSL. Citing these methods does not establish that a RangedIK-to-DLS cascade was executed in the recorded hardware experiments; the verified latest cohort remains RangedIK v7 direct.

The following references use IEEE style.

[1] Y. Wang, P. Praveena, D. Rakita, and M. Gleicher, “RangedIK: An Optimization-based Robot Motion Generation Method for Ranged-Goal Tasks,” in *Proc. 2023 IEEE International Conference on Robotics and Automation (ICRA)*, London, United Kingdom, 2023, pp. 9700–9706, doi: [10.1109/ICRA48891.2023.10161311](https://doi.org/10.1109/ICRA48891.2023.10161311). [Author project page](https://graphics.cs.wisc.edu/Papers/2023/WPRG23/) · [Open preprint](https://arxiv.org/abs/2302.13935) · [Implementation](https://github.com/uwgraphics/relaxed_ik_core/tree/ranged-ik).

[2] C. W. Wampler II, “Manipulator Inverse Kinematic Solutions Based on Vector Formulations and Damped Least-Squares Methods,” *IEEE Transactions on Systems, Man, and Cybernetics*, vol. SMC-16, no. 1, pp. 93–101, Jan. 1986, doi: [10.1109/TSMC.1986.289285](https://doi.org/10.1109/TSMC.1986.289285).

[3] Y. Nakamura and H. Hanafusa, “Inverse Kinematic Solutions With Singularity Robustness for Robot Manipulator Control,” *Journal of Dynamic Systems, Measurement, and Control*, vol. 108, no. 3, pp. 163–171, Sep. 1986, doi: [10.1115/1.3143764](https://doi.org/10.1115/1.3143764).

[Download BibTeX](references.bib). Third-party code and model licensing are separate from scholarly citation; retained notices are described in [figure sources](FIGURE_SOURCES.md).
