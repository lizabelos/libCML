import math

import numpy as np

from evo.core import sync, metrics
from evo.core.metrics import StatisticsType
from evo.core.metrics import PoseRelation
from evo.tools import file_interface
from evo.core.trajectory import PoseTrajectory3D
from evo.tools.file_interface import csv_read_matrix
from evo.tools.settings import SETTINGS


class Evaluator:

    def __init__(self, t, g):
        self.t = t
        self.g = g

    def associate(self):
        _, self.t = sync.associate_trajectories(self.g, self.t, max_diff=0.01, first_name="reference",snd_name="estimate")

    def align(self):
        self.t.align(self.g, correct_scale=True, correct_only_scale=False, n=-1)

    def ape_rmse(self):
        data = (self.g, self.t)
        ape_metric = metrics.APE(PoseRelation.full_transformation)
        ape_metric.process_data(data)
        return ape_metric.get_statistic(StatisticsType.rmse)

    def rpe_rmse(self):
        data = (self.g, self.t)
        rpe_metric = metrics.RPE(PoseRelation.full_transformation)
        rpe_metric.process_data(data)
        return rpe_metric.get_statistic(StatisticsType.rmse)

    def plot(self, dataset_name, filename):
        import numpy as np
        from evo.tools import plot
        import matplotlib.pyplot as plt
        import matplotlib.cm as cm

        plot_collection = plot.PlotCollection("evo_traj - trajectory plot")
        fig_xyz, axarr_xyz = plt.subplots(3, sharex="col", figsize=tuple(SETTINGS.plot_figsize))
        fig_rpy, axarr_rpy = plt.subplots(3, sharex="col", figsize=tuple(SETTINGS.plot_figsize))
        fig_traj = plt.figure(figsize=tuple(SETTINGS.plot_figsize))

        plot_mode = plot.PlotMode[SETTINGS.plot_mode_default]
        ax_traj = plot.prepare_axis(fig_traj, plot_mode)

        # for x-axis alignment starting from 0 with --plot_relative_time
        start_time = None
        trajectories = [self.t]

        short_traj_name = dataset_name
        plot.traj(ax_traj, plot_mode, self.g,
                      style=SETTINGS.plot_reference_linestyle,
                      color=SETTINGS.plot_reference_color,
                      label=short_traj_name,
                      alpha=SETTINGS.plot_reference_alpha)
        plot.draw_coordinate_axes(ax_traj, self.g, plot_mode,
                                      SETTINGS.plot_axis_marker_scale)
        plot.traj_xyz(
                axarr_xyz, self.g, style=SETTINGS.plot_reference_linestyle,
                color=SETTINGS.plot_reference_color, label=short_traj_name,
                alpha=SETTINGS.plot_reference_alpha,
                start_timestamp=start_time)
        plot.traj_rpy(
                axarr_rpy, self.g, style=SETTINGS.plot_reference_linestyle,
                color=SETTINGS.plot_reference_color, label=short_traj_name,
                alpha=SETTINGS.plot_reference_alpha,
                start_timestamp=start_time)

        cmap_colors = None
        if SETTINGS.plot_multi_cmap.lower() != "none":
            cmap = getattr(cm, SETTINGS.plot_multi_cmap)
            cmap_colors = iter(cmap(np.linspace(0, 1, len(trajectories))))

        for traj in trajectories:
            if cmap_colors is None:
                color = next(ax_traj._get_lines.prop_cycler)['color']
            else:
                color = next(cmap_colors)

            short_traj_name = "estimate"
            plot.traj(ax_traj, plot_mode, traj,
                      SETTINGS.plot_trajectory_linestyle, color,
                      short_traj_name, alpha=SETTINGS.plot_trajectory_alpha)
            plot.draw_coordinate_axes(ax_traj, traj, plot_mode,
                                      SETTINGS.plot_axis_marker_scale)
            plot.traj_xyz(axarr_xyz, traj, SETTINGS.plot_trajectory_linestyle,
                          color, short_traj_name,
                          alpha=SETTINGS.plot_trajectory_alpha,
                          start_timestamp=start_time)
            plot.traj_rpy(axarr_rpy, traj, SETTINGS.plot_trajectory_linestyle,
                          color, short_traj_name,
                          alpha=SETTINGS.plot_trajectory_alpha,
                          start_timestamp=start_time)
            if not SETTINGS.plot_usetex:
                fig_rpy.text(0., 0.005, "euler_angle_sequence: {}".format(
                    SETTINGS.euler_angle_sequence), fontsize=6)

        plot_collection.add_figure("trajectories", fig_traj)
        plot_collection.add_figure("xyz_view", fig_xyz)
        plot_collection.add_figure("rpy_view", fig_rpy)

        plot_collection.export(filename, confirm_overwrite=False)


def read_tum_trajectory_file2(file_path, groundtruth_path):
    """
    parses trajectory file in TUM format (timestamp tx ty tz qx qy qz qw)
    :param file_path: the trajectory file path (or file handle)
    :return: trajectory.PoseTrajectory3D object
    """
    raw_mat_est = csv_read_matrix(file_path, delim=" ", comment_str="#")
    raw_mat_gt = csv_read_matrix(groundtruth_path, delim=" ", comment_str="#")

    lines_to_keep = []
    num_removed = 0
    if (len(raw_mat_est) != len(raw_mat_gt)):
        raise Exception("The file must contains the same number of lines")

    for i in range(0, len(raw_mat_est)):
        res = True
        for v in raw_mat_gt[i]:
            if not math.isfinite(float(v)):
                res = False
                num_removed = num_removed + 1
                break
        lines_to_keep.append(res)

    print("Removing " + str(num_removed) + " lines")

    raw_mat_gt = [raw_mat_gt[i] for i in range(0, len(raw_mat_gt)) if lines_to_keep[i]]
    raw_mat_est = [raw_mat_est[i] for i in range(0, len(raw_mat_est)) if lines_to_keep[i]]


    mat_est = np.array(raw_mat_est).astype(float)
    mat_gt = np.array(raw_mat_gt).astype(float)

    stamps_est = mat_est[:, 0]  # n x 1
    xyz_est = mat_est[:, 1:4]  # n x 3
    quat_est = mat_est[:, 4:]  # n x 4
    quat_est = np.roll(quat_est, 1, axis=1)  # shift 1 column -> w in front column

    stamps_gt = mat_gt[:, 0]  # n x 1
    xyz_gt = mat_gt[:, 1:4]  # n x 3
    quat_gt = mat_gt[:, 4:]  # n x 4
    quat_gt = np.roll(quat_gt, 1, axis=1)  # shift 1 column -> w in front column

    return PoseTrajectory3D(xyz_est, quat_est, stamps_est), PoseTrajectory3D(xyz_gt, quat_gt, stamps_gt)


def loadtrajectories(context):
    reference = None
    estimate = None
    if context.d.type() == "tum":
        if context.d.g is not None:
            estimate, reference = read_tum_trajectory_file2(context.outputtum(), context.d.g)
        else:
            estimate = file_interface.read_tum_trajectory_file(context.outputtum())
    elif context.d.type() == "kitti":
        estimate = file_interface.read_kitti_poses_file(context.outputkitti())
        if context.d.g is not None:
            reference = file_interface.read_kitti_poses_file(context.d.g)
    #elif context.d.type() == "euroc":
    #    estimate = file_interface.read_euroc_csv_trajectory(csv_file)
    #    if context.d.g is not None:
    #        reference = file_interface.read_euroc_csv_trajectory(context.d.g)

    return reference, estimate


def fromslam(context):
    reference, estimate = loadtrajectories(context)
    return Evaluator(reference, estimate)
