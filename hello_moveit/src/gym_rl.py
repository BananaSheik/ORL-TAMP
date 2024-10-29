import gym
import numpy as np
import tensorflow as tf
from collections import deque
import random
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from std_msgs.msg import Float32MultiArray
import matplotlib.pyplot as plt
from IPython.display import clear_output

class PushObjectEnv(gym.Env):
    def __init__(self, node):
        if not isinstance(node, Node):
            raise TypeError(f"Expected an instance of rclpy.node.Node, got {type(node)}")

        self.node = node
        self.pose_publisher = self.node.create_publisher(Pose, 'target_pose', 10)
        self.object_position_subscriber = self.node.create_subscription(Pose, "object_position", self.object_position_callback, 10)

        self.state_space = gym.spaces.Box(low=np.array([-10.0, -10.0, -10.0, -10.0, -10.0, -10.0, -np.pi, -np.pi, -np.pi, -10.0], dtype=np.float64),
                                          high=np.array([10.0, 10.0, 10.0, 10.0, 10.0, 10.0, np.pi, np.pi, np.pi, 10.0], dtype=np.float64),
                                          dtype=np.float64)

        self.action_space = self._load_poses_from_file("/home/user/ws_moveit2/src/hello_moveit/src/valid_poses3.txt")
        print(f"target_f shape: {self.action_space.shape}")
        self.object_position = np.array([0.445993, 0.343144, 0.319642], dtype=np.float64)
        self.robot_position = np.array([0.0, 0.0, 0.0], dtype=np.float64)
        self.robot_orientation = np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64)
        self.object_position_received = False

    def step(self, action_index , step_number):
        action = self.action_space[action_index]
        pose = Pose()
        pose.position.x = action[0]
        pose.position.y = action[1]
        pose.position.z = action[2]
        pose.orientation.x = action[3]
        pose.orientation.y = action[4]
        pose.orientation.z = action[5]
        pose.orientation.w = action[6]
        self.pose_publisher.publish(pose)

        while not self.object_position_received:
            print("waiting for object position")
            rclpy.spin_once(self.node)

        print("got object position")
        self.object_position_received = False
        self.robot_position = action[:3]
        self.robot_orientation = action[3:7]

        state = np.concatenate((self.object_position, self.robot_position, self.robot_orientation))

        reward = self._calculate_reward(state)

        if np.linalg.norm(self.object_position[:2]) >= 0.6:
            print(np.linalg.norm(self.object_position[:2]))
            done = True
        elif step_number >50:
            done = True
        else:
            done = False
        return state, reward, done, {}

    def reset(self):
        self.object_position = np.array([0.5, 0.5, 0.5], dtype=np.float64)
        self.robot_position = np.random.uniform(-5.0, 5.0, size=3).astype(np.float64)
        self.robot_orientation = np.random.uniform(-np.pi, np.pi, size=4).astype(np.float64)
        self.robot_orientation /= np.linalg.norm(self.robot_orientation)
        return np.concatenate((self.object_position, self.robot_position, self.robot_orientation))

    def _calculate_reward(self, state):
        target_distance =  0.9
        object_distance = np.linalg.norm(state[:3])
        reward = -np.abs(object_distance - target_distance)

        return reward

    def _load_poses_from_file(self, file_path):
        poses = []
        with open(file_path, "r") as file:
            position = None
            orientation = None
            for line in file:
                if line.startswith("Position:"):
                    position = [float(x) for x in line.split("(")[1].split(")")[0].split(",")]
                elif line.startswith("Orientation:"):
                    orientation = [float(x) for x in line.split("(")[1].split(")")[0].split(",")]
                    poses.append(position + orientation)
        return np.array(poses, dtype=np.float64)

    def object_position_callback(self, msg):
        self.object_position = np.array([msg.position.x, msg.position.y, msg.position.z], dtype=np.float64)
        self.object_position_received = True

class PushObjectAgent:
    def __init__(self, env, gamma=0.99, epsilon=1.0, epsilon_decay=0.995, epsilon_min=0.01, batch_size=32, memory_size=10000):
        self.env = env
        self.gamma = gamma
        self.epsilon = epsilon
        self.epsilon_decay = epsilon_decay
        self.epsilon_min = epsilon_min
        self.batch_size = batch_size
        self.memory = deque(maxlen=memory_size)

        self.model = self._build_model()

    def _build_model(self):
        input_shape = self.env.state_space.shape[0]

        model = tf.keras.Sequential([
            tf.keras.layers.Input(shape=(input_shape,), dtype=tf.float64),
            tf.keras.layers.Dense(64, activation='relu'),
            tf.keras.layers.Dense(64, activation='relu'),
            tf.keras.layers.Dense(self.env.action_space.shape[0], activation='sigmoid')
        ])

        model.compile(optimizer='adam', loss='mse')
        return model

    def train(self, num_episodes):
        episode_rewards = []
        for episode in range(num_episodes):
            state = self.env.reset()
            done = False
            episode_reward = 0
            step_number = 0
            while not done:
                step_number+=1
                print(step_number)
                action_index = self.act(state)
                next_state, reward, done, _ = self.env.step(action_index , step_number)
               
                print(reward)

                episode_reward += reward
                #self.remember(state, action, reward, next_state, done)
                target = reward + self.gamma * (np.amax(self.model.predict(next_state.reshape(1, -1)), axis=1) * (1 - done))
                target = np.expand_dims(target, axis=-1)

                # Predict current Q-values
                target_f = self.model.predict(state.reshape(1, -1))
                print(f"target_f shape: {target_f.shape}")
                # Update Q-value for chosen action
                target_f[0, action_index] = target[0, 0]
                
                # self.replay()
                state = next_state
                if self.epsilon > self.epsilon_min:
                    self.epsilon *= self.epsilon_decay
            
            episode_rewards.append(episode_reward)

            clear_output(wait=True)
    
            # Plot the rewards
            plt.figure(figsize=(10, 5))
            plt.plot(range(1, len(episode_rewards) + 1), episode_rewards, label='Episode Reward')
            plt.xlabel('Episode Number')
            plt.ylabel('Episode Reward')
            plt.title('Episode Reward per Episode')
            plt.legend()
            plt.grid(True)
            plt.show(block=False)

    def act(self, state):
        
        if np.random.rand() <= self.epsilon:
            self.action_index = np.random.randint(0, len(self.env.action_space))
        else:
            q_values = self.model.predict(np.expand_dims(state, axis=0))
            self.action_index = np.argmax(q_values[0])
        return self.action_index

    def remember(self, state, action, reward, next_state, done):
        self.memory.append((state, action, reward, next_state, done))
        print("added to memeory buffer")

    def replay(self):
        if len(self.memory) < self.batch_size:
            return
        batch = random.sample(self.memory, self.batch_size)

        states = np.array([item[0] for item in batch])
        actions = np.array([item[1] for item in batch], dtype=np.int32)
        rewards = np.array([item[2] for item in batch])
        next_states = np.array([item[3] for item in batch])
        dones = np.array([item[4] for item in batch])

        targets = rewards + self.gamma * (np.amax(self.model.predict(next_states), axis=1) * (1 - dones))
        targets = np.expand_dims(targets, axis=-1)
        target_f = self.model.predict(states)

        target_f[np.arange(self.batch_size), actions[:, 0]] = targets[:, 0]

        self.model.fit(states, target_f, epochs=1, verbose=0)

        if self.epsilon > self.epsilon_min:
            self.epsilon *= self.epsilon_decay

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("push_object_node")
    try:
        env = PushObjectEnv(node)
        agent = PushObjectAgent(env)
        agent.train(num_episodes=1000)

        state = env.reset()
        while True:
            action = agent.act(state)
            next_state, reward, done, _ = env.step(action)
            state = next_state
            node.get_logger().info(f"Object position: {env.object_position}")
            if done:
                break
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
