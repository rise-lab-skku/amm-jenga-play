# amm-jenga-play
Jenga play with Franka Reasearch 3 [SKKU 2023 URP Team2's Project]

------
## Members
| [신태하](https://github.com/mirinae3145) | [신현수](https://github.com/Hyun-soo-Shin) | [이윤서](https://github.com/Corinyi) |
| :-: | :-: | :-: |
| <img src="https://avatars.githubusercontent.com/mirinae3145" width="100"> | <img src="https://avatars.githubusercontent.com/Hyun-soo-Shin" width="100"> | <img src="https://avatars.githubusercontent.com/Corinyi" width="100"> |

------
## Block Recognition

### Get **RGB Image & Depth Image** from RGB-D Camera (Intel® RealSense™ Depth Camera D435f)
| RGB Image | Depth Image |
| :-: | :-: |
| <img src="./imgs/color.png" width="400"> | <img src="./imgs/depth.png" width="400"> |

### Get each Block's Mask from RGB Image
| Extract Red | Red Mask | One Block Color | One Block Mask |
| :-: | :-: | :-: | :-: |
| <img src="./imgs/red_extract.png" width="400"> | <img src="./imgs/red_mask.png" width="400"> | <img src="./imgs/red_block2_color.png" width="400"> | <img src="./imgs/red_block2_mask.png" width="400"> |

| Blocks Merge | Masks Merge |
| :-: | :-: |
| <img src="./imgs/tower_color.png" width="400"> | <img src="./imgs/tower_mask.png" width="400"> |

### Get Point Cloud of Jenga Tower / Blocks
| Tower Point Cloud | Block Point Cloud  |
| :-: | :-: |
| <img src="./imgs/pcd_tower_before_outlier.png" width="400"> | <img src="./imgs/single_block_pcd.png" width="400"> |

### Iterative Closest Point Registration (Camera Coordinate System to Mesh Coordinate System)
| Source Point Cloud (Outlier Removed) | Target Point Cloud  |
| :-: | :-: |
| <img src="./imgs/pcd_tower.png" width="400"> | <img src="./imgs/target_pcd.png" width="400"> |

| Before ICP | After ICP  |
| :-: | :-: |
| <img src="./imgs/before_icp2.png" width="400"> | <img src="./imgs/after_icp.png" width="400"> |

### Get Block Coordinate and Target Coordinate (Mesh Coordinate System)
| Source Point Cloud (Outlier Removed) | Target Point Cloud  |
| :-: | :-: |
| <img src="./imgs/block_pcd_center_target.png" width="400"> | <img src="./imgs/tower_pcd_target.png" width="400"> |
Estimated Block Center Point (Red) and TCP Target Point (Red)