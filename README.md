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
| <img src="./block_recog/test_imgs/color_new.png" width="400"> | <img src="./block_recog/test_imgs/depth_new.png" width="400"> |

### Get each Block's Mask from RGB Image
| Extract Red | Red Mask | One Block Color | One Block Mask |
| :-: | :-: | :-: | :-: |
| <img src="./block_recog/test_imgs/red_filtered.png" width="400"> | <img src="./block_recog/test_imgs/red_masked.png" width="400"> | <img src="./block_recog/test_imgs/red_block1_rgb.png" width="400"> | <img src="./block_recog/test_imgs/red_block1_mask.png" width="400"> |

| Blocks Merge | Masks Merge |
| :-: | :-: |
| <img src="./block_recog/test_imgs/tower_color.png" width="400"> | <img src="./block_recog/test_imgs/tower_mask.png" width="400"> |

### 