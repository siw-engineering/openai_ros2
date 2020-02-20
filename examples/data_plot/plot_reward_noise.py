import numpy as np
import json
import sqlite3
from enum import Enum, auto
from matplotlib import pyplot as plt


class ArmState(Enum):
    Reached = auto()
    InProgress = auto()
    ApproachJointLimits = auto()
    Collision = auto()
    Timeout = auto()
    Undefined = auto()


def adapt_np_array(arr: np.ndarray):
    return json.dumps(arr.tolist())


def convert_np_array(text):
    return np.array(json.loads(text))


def adapt_arm_state(state: ArmState):
    return str(state.name)


def convert_arm_state(state):
    key = state.decode('utf-8')
    return ArmState[key]


table_name = 'run_19_02_2020__14_36_56'
""" create a database connection to a SQLite database """
conn = None
try:
    conn = sqlite3.connect('/home/pohzhiee/spinningup/env_log_old.db', detect_types=sqlite3.PARSE_DECLTYPES)
    print(f'Using sqlite3, version: {sqlite3.sqlite_version}')
except sqlite3.Error as e:
    print(e)

sqlite3.register_adapter(np.ndarray, adapt_np_array)
sqlite3.register_converter("np_array", convert_np_array)
sqlite3.register_adapter(ArmState, adapt_arm_state)
sqlite3.register_converter("armstate", convert_arm_state)

cur = conn.cursor()
sql_statement = \
    '''
    select cum_unshaped_reward, cum_rew_noise, cum_reward, step_num 
    from run_19_02_2020__14_36_56 
    where arm_state != "InProgress" and step_num > 50
    '''
cur.execute(sql_statement)
data = cur.fetchall()
data2 = [*zip(*data)]
# data[0] is cum_unshaped_reward
# data[1] is cum_rew_noise
# data[2] is cum_rew
# data[3] is step_num
cum_unshaped_rew = np.array(data2[0])
cum_rew_noise = np.array(data2[1])
cum_rew = np.array(data2[2])
step_nums = np.array(data2[3])

plt.figure(0)
plt.hist(cum_rew_noise, 100)
plt.title('Cumulative reward noise')

plt.figure(1)
plt.hist(cum_unshaped_rew, 100)
plt.title('Cumulative reward no noise')

plt.figure(2)
plt.hist(cum_rew, 100)
plt.title('Cumulative reward with noise')

plt.figure(3)
plt.hist(step_nums, 50)
plt.title('Steps per episode')

cum_rew_noise_std = np.std(cum_rew_noise)
print(f"Cumulative reward noise std: {cum_rew_noise_std}")

## Good reference SQL statement to get first of every distinct
## This is used to get the first/tenth/whatever reward of every episode
## This requires python 3.7 and above, python 3.6 bundled sqlite client targets sqlite 3.22 which doesn't support window functions (sqlite>3.25)
## https://stackoverflow.com/questions/16847574/how-to-use-row-number-in-sqlite
# sql_statement_rew_noise = \
# '''SELECT
#     rew_noise
# FROM (
#     SELECT
#         rew_noise,
#         ROW_NUMBER() OVER (
#             PARTITION BY episode_num
#             ORDER BY id
#         ) RowNum
#     FROM
#         run_19_02_2020__14_36_56 )
# WHERE
#     RowNum = 10;
# '''
# cur.execute(sql_statement_rew_noise)
# data_n = cur.fetchall()
#
# rew_noise_1 = np.array(data_n)
# rew_noise_1_std = np.std(rew_noise_1)
# print(f"Reward noise step 10 std: {rew_noise_1_std}")
# plt.figure(5)
# plt.hist(rew_noise_1, 100)


plt.show()
print('hello')
