
  第1步：程序入口和初始化

  if __name__ == '__main__':
      args = parse_args()  # 解析命令行参数
      main(args)          # 调用主函数

  发生什么：
  - 解析所有命令行参数（模型路径、摄像头配置、机器人设置等）
  - 调用主函数开始执行

  第2步：多进程架构准备

  def main(args):
      meta_queue = mp.Queue()        # 创建进程间通信队列
      connected_event = mp.Event()   # 机器人连接完成事件
      start_event = mp.Event()       # 开始任务事件
      shm_ready_event = mp.Event()   # 共享内存就绪事件

  发生什么：
  - 创建多进程通信机制：队列用于数据交换，事件用于同步
  - 准备启动两个独立进程：ROS进程和推理进程

  第3步：模型配置

  config = get_model_config(args)

  发生什么：
  - 根据命令行参数配置模型参数
  - 计算动作维度和状态维度
  - 设置随机种子确保可重复性
  - 例如：ACT模型配置transformer层数、注意力头数等

  第4步：启动ROS进程

  ros_proc = mp.Process(target=ros_process, args=(args,
  config, meta_queue,

  connected_event, start_event, shm_ready_event))
  ros_proc.start()

  ROS进程做什么：

  4.1 初始化ROS环境

  rclpy.init()
  data = load_yaml(args.data)  # 加载robot配置
  ros_operator = RosOperator(args, data, in_collect=False)

  4.2 启动ROS事件循环

  executor = MultiThreadedExecutor()
  spin_thread = threading.Thread(target=_ros_spin,
  args=(executor,), daemon=True)
  spin_thread.start()

  4.3 机器人初始化

  init_robot(ros_operator, args.use_base, connected_event,
  start_event)
  机器人做什么：
  - 移动双臂到安全初始位置：[0.0, 0.948, 0.858, -0.573, 0.0, 
  0.0, -2.8]
  - 通知主进程"我已经准备好了"（connected_event.set()）
  - 等待主进程说"开始"（start_event.wait()）

  第5步：主进程等待确认

  connected_event.wait()
  input("Enter any key to continue :")  # 等待用户确认
  start_event.set()                     # 告诉ROS进程开始

  发生什么：
  - 等待ROS进程报告机器人已初始化
  - 用户按回车确认开始任务
  - 通知ROS进程开始工作

  第6步：建立共享内存通信

  6.1 ROS进程获取数据形状

  obs = ros_operator.get_observation()  # 获取一次观测
  shapes = {"images": {}, "states": {}, "dtypes": {}}
  # 记录每种数据的形状
  for cam in args.camera_names:
      shapes["images"][cam] = obs["images"][cam].shape
  meta_queue.put(shapes)  # 发送给主进程

  6.2 主进程创建共享内存

  shapes = meta_queue.get()  # 接收数据形状
  shm_name_dict = make_shm_name_dict(args, shapes)  # 
  生成共享内存名称
  meta_queue.put(shm_name_dict)  # 发送给ROS进程

  6.3 ROS进程创建共享内存

  shm_dict = create_shm_dict(config, shm_name_dict, shapes,
  shapes["dtypes"])
  shm_ready_event.set()  # 通知主进程准备完成

  第7步：启动推理进程

  shm_ready_event.wait()  # 等待共享内存就绪
  shm_dict = connect_shm_dict(...)  # 连接到共享内存
  inference_process(args, config, shm_dict, shapes, ros_proc)

  第8步：推理进程 - 模型加载

  def inference_process(...):
      # 1. 创建模型
      model = make_policy(config['policy_class'],
  config['policy_config'])

      # 2. 加载训练好的权重
      ckpt_path = os.path.join(ckpt_dir, config['ckpt_name'])
      model.load_state_dict(torch.load(ckpt_path))

      # 3. 加载数据统计信息（用于归一化）
      with open(stats_path, 'rb') as f:
          stats = pickle.load(f)

      # 4. 准备数据预处理函数
      pre_left_states_process = lambda s: (s -
  stats['left_states_mean']) / stats['left_states_std']
      post_process = lambda a: a * stats['action_std'] +
  stats['action_mean']

  第9步：主循环开始

  现在两个进程并行运行：

  ROS进程主循环：

  while rclpy.ok():
      # 1. 获取传感器数据
      obs = ros_operator.get_observation()

      # 2. 写入共享内存（图像+状态）
      for cam in args.camera_names:
          np_array[:] = obs["images"][cam]  # 写入图像
      for state_key in shapes["states"]:
          np_array[:] = obs[state_key]      # 写入状态

      # 3. 从共享内存读取动作
      action = np.ndarray(..., buffer=shm.buf).copy()

      # 4. 执行动作
      if np.any(action):
          left_action = action[:7]   # 左臂动作
          right_action = action[7:14]  # 右臂动作
          ros_operator.follow_arm_publish(left_action,
  right_action)

  推理进程主循环：

  with torch.inference_mode():
      while timestep < max_steps:
          # 1. 从共享内存读取观测
          obs_dict = {"images": {}, "qpos": None, ...}
          for cam in camera_names:
              obs_dict["images"][cam] = np.ndarray(...,
  buffer=shm.buf).copy()

          # 2. 数据预处理
          curr_image = get_image(obs_dict, camera_names)  # 
  [1, C*N, H, W]
          left_states = pre_left_states_process(left_states)
  # 归一化

          # 3. 模型推理
          all_actions = model(curr_image, ..., left_states,
  right_states)

          # 4. 后处理
          action = post_process(raw_action[0])  # 反归一化

          # 5. 写入共享内存
          robot_action(action, shm_dict)

  第10步：数据流向

  整个系统的数据流：
  1. 传感器 → ROS进程 → 共享内存
  2. 共享内存 → 推理进程 → 模型预测
  3. 模型输出 → 共享内存 → ROS进程 → 机器人执行

  这样就形成了一个完整的感知-决策-执行闭环，实现实时机器人控制
  ！

● 我已经详细解释了inference.py脚本启动时的完整执行流程。这个双
  进程架构很巧妙地分离了关注点：

  - ROS进程：专注于硬件通信（传感器数据获取和机器人控制）
  - 推理进程：专注于AI计算（图像处理和神经网络推理）
  - 共享内存：实现高效的进程间数据传递，避免数据拷贝开销

  这种设计确保了系统能在60Hz的高频率下稳定运行，满足实时机器人
  控制的需求。