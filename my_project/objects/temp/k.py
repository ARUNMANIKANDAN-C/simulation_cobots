import random
import numpy as np
from collections import deque
import tensorflow as tf
from tensorflow.keras.applications import MobileNet
from tensorflow.keras.layers import Dense, Flatten
from tensorflow.keras.models import Model
from hyper import step , reset
# Hyperparameters
gamma = 0.99            # Discount factor
epsilon = 1.0           # Initial exploration rate
epsilon_min = 0.1       # Minimum exploration rate
epsilon_decay = 0.995   # Decay rate for exploration
batch_size = 32         # Batch size for training
num_episodes = 1000     # Total training episodes
memory_size = 2000      # Replay memory size
num_actions = 4         # Set this to the actual number of actions in your Webots environment

# Replay buffer
memory = deque(maxlen=memory_size)

# Preprocess function for Webots state (resize, normalize, etc.)
def preprocess_state(state):
    # Adjust preprocessing as needed based on your Webots observations
    state = tf.image.resize(state, (224, 224))  # Resize to MobileNet input size
    state = np.expand_dims(state, axis=0)       # Add batch dimension
    return state / 255.0                        # Normalize pixel values

# MobileNet feature extractor with a DQN output layer
mobilenet_base = MobileNet(weights="imagenet", include_top=False, input_shape=(224, 224, 3))

# Freeze MobileNet layers
for layer in mobilenet_base.layers:
    layer.trainable = False

# Custom DQN layers on top of MobileNet
x = Flatten()(mobilenet_base.output)
x = Dense(256, activation='relu')(x)
q_values = Dense(num_actions, activation='linear')(x)

# Final model
model = Model(inputs=mobilenet_base.input, outputs=q_values)
model.compile(optimizer='adam', loss='mse')

# Epsilon-greedy action selection
def get_action(state):
    if np.random.rand() <= epsilon:
        return random.randrange(num_actions)  # Random action
    q_values = model.predict(state)  # Predict Q-values
    return np.argmax(q_values[0])    # Action with highest Q-value

# Experience replay function
def replay():
    global epsilon
    if len(memory) < batch_size:
        return

    minibatch = random.sample(memory, batch_size)
    states, actions, rewards, next_states, dones = zip(*minibatch)

    states = np.array(states)
    next_states = np.array(next_states)

    # Q-values for current states and next states
    q_values = model.predict(states)
    q_values_next = model.predict(next_states)

    # Update Q-values for each experience in the batch
    for i in range(batch_size):
        if dones[i]:
            q_values[i][actions[i]] = rewards[i]
        else:
            q_values[i][actions[i]] = rewards[i] + gamma * np.amax(q_values_next[i])

    # Train model on the batch
    model.fit(states, q_values, epochs=1, verbose=0)

    # Decay epsilon
    if epsilon > epsilon_min:
        epsilon *= epsilon_decay

# Store experiences in replay memory
def store_experience(state, action, reward, next_state, done):
    memory.append((state, action, reward, next_state, done))

# Training loop
for episode in range(num_episodes):
    state = reset()  # Reset environment at the start of each episode
    state = preprocess_state(state)
    done = False
    total_reward = 0

    while not done:
        action = get_action(state)
        next_state, reward, done = step(action)  # Interact with Webots environment
        next_state = preprocess_state(next_state)

        # Store experience and train
        store_experience(state, action, reward, next_state, done)
        replay()

        # Move to the next state
        state = next_state
        total_reward += reward

    print(f"Episode {episode + 1}/{num_episodes} - Total Reward: {total_reward}, Epsilon: {epsilon}")

# Save the model
model.save('mobilenet_dqn_webots.h5')
