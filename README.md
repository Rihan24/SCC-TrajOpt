# Statistical Contraction for Chance-Constrained Trajectory Optimization of Non-Gaussian Stochastic Systems
Codes for simulation and hardware experiments carried out in the paper, "[Statistical Contraction for Chance-Constrained Trajectory Optimization of Non-Gaussian Stochastic Systems](https://arxiv.org/abs/2603.07092)", by Rihan Aaron D'Silva and Hiroyasu Tsukamoto.

## Requirements
Dependencies include ```torch```, ```casadi```, ```tqdm```, ```numpy```, and ```matplotlib```. You can install them using the following command.
```bash
pip install -r requirements.txt
```

## Usage
We employ Neural contraction metrics and tracking controller from the implementation in "[Learning Certified Control Using Contraction Metric](https://github.com/sundw2014/C3M)". To confirm working, run the pretrained controller using the command,

```
python plot.py --pretrained log_QUADROTORM_R100_0.5_25_0.8/controller_best.pth.tar --task QUADROTOR_9D --plot_type 3D --plot_dims 0 1 2
```


<!--
If you find this project useful, please cite:
```bibtex
@article{sun2020learning,
  title = {Learning certified control using contraction metric},
  author = {Sun, Dawei and Jha, Susmit and Fan, Chuchu},
  booktitle = {Proceedings of the Conference on Robot Learning},
  year = {2020}
}
```
-->
