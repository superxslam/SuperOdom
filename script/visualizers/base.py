
import abc
import logging

import torch

import utils

class SemanticRgbdVis(abc.ABC):
  """Base interface for all Semantic RGBD Visualizers"""
  def __init__(self, intrinsics_3x3: torch.FloatTensor, img_size = None,
               base_point_size: float = None,
               global_heat_scale: bool = False,
               pose_period: int = 1, img_period: int = 1, pc_period: int = 1,
               feat_proj_basis_path: str = None, device: str = None):
    """
    Args:
      intrinsics_3x3: A 3x3 float tensor or including camera intrinsics.
      img_size: If not set to None, then all 2D images will be resized before
        being logged. Useful to reduce memory requirement or to ensure all 2D
        images being logged have the same size and are aligned.
      base_point_size: size of points/voxels in world units. Set to None to
        leave it up to the visualizer to set a reasonable default.
      global_heat_scale: Whether to use the same heatmap scale across all logs.
      pose_period: How often to log pose. Set to -1 to disable logging pose.
      img_period: How often to log images (RGB, Depth, Features, labels...etc.). 
        Set to -1 to disable logging all 2D images.
      pc_period: How often to point clouds (RGB, features, labels...etc.).
        Set to -1 to disable logging all 3D point clouds.
      feat_proj_basis_path: A .pt path to a CxD float tensor that has the basis
        to use for projecting features to RGB space for visualization. Only
        first 3 PCA components are used [:, :3].
        if set to None, then a PCA basis will be computed on the first call to
        either log_feature_pc or log_feature_image and used for all subsequent
        calls.
    """
    self.intrinsics_3x3 = intrinsics_3x3
    self.img_size = img_size
    self.base_point_size = base_point_size
    self.global_heat_scale = global_heat_scale
    self.pose_period = pose_period if pose_period > 0 else torch.inf
    self.img_period = img_period if img_period > 0 else torch.inf
    self.pc_period = pc_period if pc_period > 0 else torch.inf

    if device is None:
      self.device = "cuda" if torch.cuda.is_available() else "cpu"
    else:
      self.device = device

    self.vis_time = 0
    self.pose_time = -1
    self.img_time = -1
    self.pc_time = -1

    self.pc_updated = False
    self.pose_updated = False
    self.img_updated = False

    if feat_proj_basis_path is None:
      self.basis = None
    else:
      self.basis = torch.load(feat_proj_basis_path, 
                              weights_only=True).to(self.device)
      if len(self.basis.shape) != 2:
        raise ValueError(f"Invalid basis loaded from {feat_proj_basis_path}. "
                         f"Expected shape CxD but found {self.basis.shape}.")
      self.basis = self.basis[:, :3]

    # Running max and min to use for normalizing the features for visualization.
    self.projected_feats_max = None
    self.projected_feats_min = None

    # Running max and min heat to use for normalizing heatmaps consistently
    self.heat_max = None
    self.heat_min = None

  def _preprocess_img(self, img: torch.FloatTensor):
    """Resize image to self.img_size if needed
    
    Args:
      img: (HxWx3) Float tensor in the range [0-1]
    
    Returns:
      (H'xW'x3) Float tensor in the range [0-1]
    """
    src_h, src_w = img.shape[:2]
    if self.img_size is not None:
      try:
        tgt_h, tgt_w = self.img_size
      except TypeError:
        tgt_h = tgt_w = self.img_size

      if tgt_h != src_h or tgt_w != src_w:
        if len(img.shape) == 3:
          img = img.permute(2, 0, 1).unsqueeze(0)
        return torch.nn.functional.interpolate(
          img, size = (tgt_h, tgt_w),
          mode = "bilinear").squeeze().permute(1, 2, 0)
    return img

  @abc.abstractmethod
  def _log_rgb_image(self,
                     rgb_img: torch.FloatTensor,
                     layer: str = "rgb"):
    """RGB image logging primitive. Must be implemented by child class.
    
    Args:
      rgb_img: (HxWx3) Float tensor in the range [0-1]
      layer: Name of layer to log to. Interpretation of layer varies by
        implementing class.
    """
    self.img_updated = True

  @abc.abstractmethod
  def _log_pose(self,
                pose_4x4: torch.FloatTensor,
                layer: str = "pose"):
    """Pose logging primitive. Must be implemented by child class.

    Args:
      pose_4x4: (4x4) float tensor. Pose is a 4x4 float32 tensor in opencv RDF.
        a pose is the extrinsics transformation matrix that takes you from
        camera/robot coordinates to world coordinates. Last row should always be
        [0, 0, 0, 1]
      layer: Name of layer to log to. Interpretation of layer varies by
        implementing class.
    """
    self.pose_updated = True

  @abc.abstractmethod
  def _log_pc(self, pc_xyz, pc_rgb = None, pc_radii = None, layer = "rgb"):
    """Point cloud logging primitive. Must be implemented by child class.

    Args:
      pc_xyz: (Nx3) float tensor
      pc_rgb: (Nx3) float tensor from 0-1 or (Nx4) for an optional alpha channel
      pc_radii: (Nx1) float tensor of values [0 - +inf] describing the radius of
        each point. The implementation may choose to ignore this.
      layer: Name of layer to log to. Interpretation of layer varies by
        implementing class.
    """
    self.pc_updated = True

  @abc.abstractmethod
  def _log_arrows(self,
                  origins: torch.FloatTensor,
                  dirs: torch.FloatTensor,
                  rgb: torch.FloatTensor = None,
                  layer: str = "arrows"):
    """Arrow logging primitive. Must be implemented by child class.
    
    Args:
      origins: (Nx3) Float tensor represeting starting points of arrows
      dirs: (Nx3) Float tensor represeting direction vectors. 
        Need not be unit vectors.
      rgb: (Nx3) (Optional) colors for arrows.
      layer: Name of layer to log to. Interpretation of layer varies by
        implementing class.
    """

  # @abc.abstractmethod
  # TODO def _log_label_image(self, pc_xyz, pc_labels = None, layer = "label"):
  #   self.img_updated = True

  @abc.abstractmethod
  def _log_label_pc(self, pc_xyz, pc_labels = None, layer = "label"):
    self.pc_updated = True
    # TODO: Provide default implementation that does category to rgb then logs
    # using the normal primitive _log_pc. _log_label_pc should NOT be an 
    # abstract primitive that must be implemented separately by children.

  def _log_heat_pc(self, pc_xyz, pc_heat = None, layer = "heat"):
    self.pc_updated = True
    pc_heat = self._normalize_heat(pc_heat).unsqueeze(-1)
    c1 = torch.tensor([253, 231, 37], dtype=torch.float, device=pc_heat.device)
    c1 = c1.unsqueeze(0)/255
    c2 = torch.tensor([68, 1, 84], dtype=torch.float, device=pc_heat.device)
    c2 = c2.unsqueeze(0)/255
    cmapped_pc = pc_heat * c1 + (1-pc_heat) * c2
    self._log_pc(pc_xyz, cmapped_pc, layer=layer)

  def _log_occ_pc(self, pc_xyz, pc_occ = None, pc_radii = None,
                  layer = "occupancy"):
    self.pc_updated = True
    color = torch.tensor([0.8, 0.2, 0.8],
                         dtype=torch.float, device=pc_occ.device)
    colors = torch.empty((pc_xyz.shape[0], 4),
                         dtype=torch.float, device=pc_occ.device)
    colors[:, :3] = color
    if pc_occ.shape[0] > 0:
      colors[:, -1:] = utils.norm_01(pc_occ)
    if pc_radii is None:
      pc_radii = (colors[:, -1]+0.2)/1.2*self.base_point_size
    self._log_pc(pc_xyz, colors, pc_radii, layer)

  def _normalize_heat(self, heat):
    if not self.global_heat_scale:
      return utils.norm_01(heat)

    mn = heat.min()
    mx = heat.max()
    if self.heat_max is None:
      self.heat_max = mx
      self.heat_min = mn
    else:
      self.heat_max = torch.maximum(mx, self.heat_max)
      self.heat_min = torch.minimum(mn, self.heat_min)

    heat = heat - self.heat_min
    scale = self.heat_max - self.heat_min
    heat = heat / scale
    return heat

  def _normalize_projected_feats(self, proj_feats):
    projected_feats_min = torch.min(
      proj_feats.reshape(-1, 3), dim=0).values
    projected_feats_max = torch.max(
      proj_feats.reshape(-1, 3), dim=0).values

    if self.projected_feats_min is None:
      self.projected_feats_max = projected_feats_max
      self.projected_feats_min = projected_feats_min
    else:
      self.projected_feats_min = torch.minimum(projected_feats_min,
                                               self.projected_feats_min)
      self.projected_feats_max = torch.maximum(projected_feats_max,
                                               self.projected_feats_max)

    proj_feats = proj_feats - self.projected_feats_min
    scale = self.projected_feats_max - self.projected_feats_min
    proj_feats = proj_feats / scale
    return proj_feats

  def _log_depth_image(self, depth_img: torch.FloatTensor, layer = "depth"):
    """
    
    Args:
      depth_img: HxW float tensor describing depth in world scale. 
    """
    self.img_updated = True
    H,W = depth_img.shape
    rgb_depth_img = torch.zeros(size=(H, W, 3), device=depth_img.device,
                                dtype=torch.float)
    mask = depth_img.isfinite()
    rgb_depth_img[mask, :] = utils.norm_01(depth_img[mask]).unsqueeze(-1)
    mask = depth_img.isposinf()
    rgb_depth_img[mask, :] = torch.tensor([1., 0, 0])
    mask = depth_img.isneginf()
    rgb_depth_img[mask, :] = torch.tensor([0, 1., 0])
    mask = depth_img.isnan()
    rgb_depth_img[mask, :] = torch.tensor([0, 0, 1.])
    self._log_rgb_image(rgb_depth_img, layer=layer)

  def _log_feature_image(self, feat_img, layer = "features"):
    H,W,C = feat_img.shape
    if C != 3:
      feat_imgs_flat = feat_img.reshape(-1, C)
      if self.basis is None or self.basis.shape[0] != C:
        if self.basis is not None:
          logging.warning("Loaded basis does not match features given. "
                          "Computing a new basis.")
        U, S, V = torch.pca_lowrank(feat_imgs_flat, q = 3)
        self.basis = V

      feat_img = feat_imgs_flat @ self.basis
      feat_img = feat_img.reshape(H, W, 3)

    feat_img = self._normalize_projected_feats(feat_img)

    self._log_rgb_image(self._preprocess_img(feat_img).cpu(), layer=layer)

  def _log_feature_pc(self, pc_xyz, pc_feat, layer = "feat"):
    N, C = pc_feat.shape
    if C != 3:
      if self.basis is None or self.basis.shape[0] != C:
        if self.basis is not None:
          logging.warning("Loaded basis does not match features given. "
                          "Computing a new basis.")
        U, S, V = torch.pca_lowrank(pc_feat, q = 3)
        self.basis = V
      pc_feat = pc_feat @ self.basis.to(pc_feat.device)
    pc_feat = self._normalize_projected_feats(pc_feat)
    self._log_pc(pc_xyz.cpu(), pc_feat.cpu(), layer=layer)


  def _log_feature_arrows(self,
                          origins: torch.FloatTensor,
                          dirs: torch.FloatTensor,
                          feats: torch.FloatTensor = None,
                          layer: str = "arrows"):
    N, C = feats.shape
    if C != 3:
      if self.basis is None or self.basis.shape[0] != C:
        if self.basis is not None:
          logging.warning("Loaded basis does not match features given. "
                          "Computing a new basis.")
        U, S, V = torch.pca_lowrank(feats, q = 3)
        self.basis = V
      feats = feats @ self.basis.to(feats.device)
    feats = self._normalize_projected_feats(feats)
    self._log_arrows(origins.cpu(), dirs.cpu(), feats.cpu(), layer=layer)

  def _log_heat_arrows(self,
                       origins: torch.FloatTensor,
                       dirs: torch.FloatTensor,
                       heat: torch.FloatTensor = None,
                       layer: str = "arrows"):
    heat = self._normalize_heat(heat).unsqueeze(-1)
    c1 = torch.tensor([253, 231, 37], dtype=torch.float, device=heat.device)
    c1 = c1.unsqueeze(0)/255
    c2 = torch.tensor([68, 1, 84], dtype=torch.float, device=heat.device)
    c2 = c2.unsqueeze(0)/255
    cmapped = heat * c1 + (1-heat) * c2
    self._log_arrows(origins.cpu(), dirs.cpu(), cmapped.cpu(), layer=layer)

  @abc.abstractmethod
  def _log_label_arrows(self,
                       origins: torch.FloatTensor,
                       dirs: torch.FloatTensor,
                       labels: torch.FloatTensor = None,
                       layer: str = "arrows"):

    self.pc_updated = True
    # TODO: Provide default implementation that does category to rgb then logs

  def log_posed_rgbd_pc(self, rgb_img = None, depth_img = None, pose_4x4 = None,
                        pc_xyz = None, pc_rgb = None, **kwargs):
    """
    Args:
      rgb_img: A HxWx3 RGB float tensor within the (0-1) range describing.
      depth_img: HxW float tensor with values > 0 describing depth images.
        May include NaN and Inf values which will be ignored.
      pose_4x4: A 4x4 tensor which includes poses in opencv RDF.
        a pose is the extrinsics transformation matrix that takes you from
        camera/robot coordinates to world coordinates.
      pc_xyz: A Nx3 point cloud with xyz values
      pc_rgb: A Nx3 or Nx4 point cloud with rgb or rgba values designating the
        colors of the point cloud pc_xyz
      kwargs: Extra images or point clouds to log. 
        For point clouds the format is pc_<layer_name> whereas for 2D images
        the format is img_<layer_name>
    """
    log_imgs = self.vis_time - self.img_time >= self.img_period
    log_pose = self.vis_time - self.pose_time >= self.pose_period
    log_pc = self.vis_time - self.pc_time >= self.pc_period
    if rgb_img is not None and log_imgs:
      self._log_rgb_image(self._preprocess_img(rgb_img).cpu())

    if depth_img is not None and log_imgs:
      self._log_depth_image(self._preprocess_img(depth_img).cpu())

    if pose_4x4 is not None and log_pose:
      self._log_pose(pose_4x4.cpu())

    if pc_xyz is not None and log_pc:
      self._log_pc(pc_xyz.cpu(), pc_rgb.cpu())

    # Extra images or point clouds.
    # TODO: Refactor into a better API
    for k,v in kwargs.items():
      layer_type, layer_name = k.split("_")
      if layer_type == "img" and log_imgs:
        self._log_rgb_image(self._preprocess_img(v).cpu(), layer=layer_name)
      elif layer_type == "pc" and log_pc:
        if len(v) == 1:
          self._log_pc(v[0].cpu(), layer=layer_name)
        else:
          self._log_pc(v[0].cpu(), v[1].cpu(), layer=layer_name)
      elif layer_type == "featimg" and log_imgs:
        # May have heavy calculations so we don't convert to CPU right away.
        self._log_feature_image(v, layer_name)
      elif layer_type == "featpc" and log_pc:
        # May have heavy calculations so we don't convert to CPU right away.
        self._log_feature_pc(v[0], v[1], layer=layer_name)
      elif layer_type == "heatpc" and log_pc:
        self._log_heat_pc(v[0].cpu(), v[1].cpu(), layer=layer_name)
      elif layer_type == "labelpc" and log_pc:
        self._log_label_pc(v[0].cpu(), v[1].cpu(), layer=layer_name)
      elif layer_type == "occpc" and log_pc:
        if len(v) == 2:
          self._log_occ_pc(v[0].cpu(), v[1].cpu(), layer=layer_name)
        else:
          self._log_occ_pc(v[0].cpu(), v[1].cpu(), v[2].cpu(),
                           layer=layer_name)
      elif layer_type == "arrows":
        if len(v) == 2:
          self._log_arrows(v[0].cpu(), v[1].cpu())
        else:
          self._log_arrows(v[0].cpu(), v[1].cpu(), v[2].cpu())
      elif layer_type == "featarrows":
        self._log_feature_arrows(v[0], v[1], v[2],
                                 layer=layer_name)

  def next_frame(self, timestamp = None):
    if self.img_updated:
      self.img_time = self.vis_time
      self.img_updated = False
    if self.pose_updated:
      self.pose_time = self.vis_time
      self.pose_updated = False
    if self.pc_updated:
      self.pc_time = self.vis_time
      self.pc_updated = False

    if timestamp is None:
      self.vis_time += 1
    else:
      if timestamp <= self.vis_time:
        raise ValueError("If setting time stamp manually. It must be greater, "
                         "than current visualization time.")
      self.vis_time = timestamp
