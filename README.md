# Statistical Contraction for Chance-Constrained Trajectory Optimization of Non-Gaussian Stochastic Systems
Codes for simulation and hardware experiments carried out in the paper, "[Statistical Contraction for Chance-Constrained Trajectory Optimization of Non-Gaussian Stochastic Systems](https://arxiv.org/abs/2011.12569)", by Rihan Aaron D'Silva and Hiroyasu Tsukamoto.

## Requirements
Dependencies include ```torch```, ```casadi```, ```tqdm```, ```numpy```, and ```matplotlib```. You can install them using the following command.
```bash
pip install -r requirements.txt
```

## Usage
The neural contraction metrics are computed using the implementation in ' The script ```main.py``` can be used for learning the controller. Usage of this script is as follows
```


For example, run the following command to learn a controller for the 8-dimensional quadrotor model.
```
mkdir log_QUADROTOR_8D
python main.py --log log_QUADROTOR_8D --task QUADROTOR_8D
```

Run the following command to evaluate the learned controller and plot the results.
```
python plot.py --pretrained log_QUADROTOR_8D/controller_best.pth.tar --task QUADROTOR_8D --plot_type 3D --plot_dims 0 1 2
python plot.py --pretrained log_QUADROTOR_8D/controller_best.pth.tar --task QUADROTOR_8D --plot_type error
```

If you find this project useful, please cite:
```bibtex
@article{sun2020learning,
  title = {Learning certified control using contraction metric},
  author = {Sun, Dawei and Jha, Susmit and Fan, Chuchu},
  booktitle = {Proceedings of the Conference on Robot Learning},
  year = {2020}
}
```
