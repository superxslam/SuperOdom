from typing_extensions import override

import rerun as rr
import torch

from base import SemanticRgbdVis
import utils

class RerunVis(SemanticRgbdVis):
  """Semantic RGBD visualizer using ReRun.io"""
  def __init__(self, intrinsics_3x3: torch.FloatTensor, img_size = None,
               base_point_size = None,
               global_heat_scale = False,
               pose_period = 1, img_period = 1, pc_period = 1,
               feat_proj_basis_path = None, split_label_vis = False):
    super().__init__(intrinsics_3x3, img_size, base_point_size,
                     global_heat_scale, pose_period, img_period, pc_period,
                     feat_proj_basis_path)

    rr.init("semantic_mapping_vis", spawn=True)
    rr.set_time_seconds("stable_time", 0)
    self.base_name = "map"

    rr.log(self.base_name, rr.ViewCoordinates.FLU, static=True)

    rr.log(self.base_name,
      rr.Arrows3D(
        vectors=[[1, 0, 0], [0, 1, 0], [0, 0, 1]],
        colors=[[255, 0, 0], [0, 255, 0], [0, 0, 255]],
      ))
    self.height = None
    self.width = None
    self.prev_pose_4x4 = None
    self.split_label_vis = split_label_vis

  @override
  def _log_rgb_image(self, rgb_img, layer = "rgb"):
    super()._log_rgb_image(rgb_img, layer)
    rr.log(f"{self.base_name}/pinhole/{layer}", rr.Image(rgb_img))

  @override
  def _log_depth_image(self, depth_img, layer = "depth"):
    self.height, self.width = depth_img.shape
    self.img_updated = True
    rr.log(f"{self.base_name}/pinhole/{layer}", rr.DepthImage(depth_img))

  @override
  def _log_pose(self, pose_4x4, layer = "pose"):
    super()._log_pose(pose_4x4, layer)
    rr.log(f"{self.base_name}/tf",
           rr.Pinhole(image_from_camera=self.intrinsics_3x3,
                      height=self.height, width=self.width,
                      camera_xyz=rr.ViewCoordinates.RDF))
    rr_transform = rr.Transform3D(translation=pose_4x4[:3, 3],
                                  mat3x3=pose_4x4[:3, :3], from_parent=False)
    rr.log(f"{self.base_name}/tf", rr_transform)
    self.prev_pose_4x4 = pose_4x4

  @override
  def _log_pc(self, pc_xyz, pc_rgb = None, pc_radii = None, layer = "rgb"):
    super()._log_pc(pc_xyz, pc_rgb, layer)
    radii = self.base_point_size if pc_radii is None else pc_radii
    rr.log(f"{self.base_name}/{layer}",
           rr.Points3D(positions=pc_xyz, colors=pc_rgb, radii=radii))

  @override
  def _log_arrows(self, origins, dirs, rgb = None, layer="arrows"):
    super()._log_arrows(origins, dirs, rgb, layer)
    rr.log(f"{self.base_name}/{layer}_arr",
           rr.Arrows3D(vectors=dirs*self.base_point_size*5, origins=origins,
                       colors=rgb, radii=self.base_point_size/3))

  @override
  def _log_label_pc(self, pc_xyz, pc_labels = None, layer = "label"):
    super()._log_label_pc(pc_xyz, pc_labels, layer)
    if self.split_label_vis and len(pc_labels.shape)==1 and pc_labels.shape[0] > 0:
      unique = torch.unique(pc_labels)
      pc_labels_onehot = torch.nn.functional.one_hot(pc_labels)
      for i in unique:
        rr.log(f"{self.base_name}/{layer}_pc/{i}", 
               rr.Points3D(positions=pc_xyz[pc_labels_onehot[..., i]==1], class_ids=i,
               radii=self.base_point_size))
    else:
      rr.log(f"{self.base_name}/{layer}_pc",
            rr.Points3D(positions=pc_xyz, class_ids=pc_labels,
                        radii=self.base_point_size))

  @override
  def _log_label_arrows(self, origins, dirs, labels, layer="arrow_labels"):
    super()._log_label_arrows(origins, dirs, labels, layer)
    if self.split_label_vis and len(labels.shape)==1 and labels.shape[0] > 0:
      unique = torch.unique(labels)
      labels_onehot = torch.nn.functional.one_hot(labels)
      for i in unique:
        m = labels_onehot[..., i]==1
        rr.log(f"{self.base_name}/{layer}_arr/{i}",
              rr.Arrows3D(vectors=(dirs*self.base_point_size*5)[m], origins=origins[m],
                          class_ids=i, radii=self.base_point_size/3))
    else:
      rr.log(f"{self.base_name}/{layer}_arr",
            rr.Arrows3D(vectors=dirs*self.base_point_size*5, origins=origins,
                        class_ids=labels, radii=self.base_point_size/3))

  def _log_box(self, box_mins, box_maxs, layer = ""):
    box_centers = (box_maxs + box_mins) / 2
    box_half_sizes = (box_maxs - box_mins) / 2
    rr.log(f"{self.base_name}/{layer}_boxes",
           rr.Boxes3D(centers=box_centers, half_sizes=box_half_sizes))


  def _log_axes(self, pose_4x4, layer="axes"):
    for i in range(pose_4x4.shape[0]):
      rr_transform = rr.Transform3D(
        translation=pose_4x4[i, :3, 3], mat3x3=pose_4x4[i, :3, :3],
        from_parent=False, axis_length=1.0,
        scale=2)
      rr.log(f"{self.base_name}/{layer}/{i}", rr_transform)

  @override
  def next_frame(self, timestamp = None):
    super().next_frame(timestamp)
    rr.set_time_seconds("stable_time", self.vis_time*0.1)