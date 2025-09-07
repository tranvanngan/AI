
#bài 1 cài đặt 5 thuật toán trên python, cố gắng tái sử dụng nhau
from collections import deque
import heapq
import time
#Node là cấu trúc dữ liệu dùng chung cho các thuật toán:
#Dùng để lưu state (ví dụ: "HELLO")
#Lưu parent để dựng lại đường đi (truy vết ngược)
#path_cost → chi phí đường đi (quan trọng cho UCS)
#depth → độ sâu (quan trọng cho DFS/DLS/IDS)
#solution() → trả về chuỗi các hành động và các state trên đường đi.
# =======================
# Node lưu trạng thái
# =======================
class Node:
    def __init__(self, state, parent=None, action=None, path_cost=0, depth=0):
        self.state = state
        self.parent = parent
        self.action = action
        self.path_cost = path_cost
        self.depth = depth

    def __lt__(self, other):
        return self.path_cost < other.path_cost

    def solution(self):
        actions, states = [], []
        node = self
        while node.parent is not None:
            actions.append(node.action)
            states.append(node.state)
            node = node.parent
        actions.reverse()
        states.reverse()
        return actions, states

#Đây là lớp trừu tượng (abstract class):
# =======================
# Lớp SearchProblem cơ bản
# =======================
class SearchProblem:
    def __init__(self, initial, goal):
        self.initial = initial    #initial → trạng thái ban đầu (ở đây là "")
        self.goal = goal     #goal → trạng thái đích ("HELLO WORLD")

    def goal_test(self, state): #goal_test(state) → kiểm tra state có phải goal không
        return state == self.goal

    def actions(self, state):
        #actions(state) → trả về các hành động hợp lệ từ state (ở đây là thêm 1 ký tự tiếp theo)
        raise NotImplementedError

    def result(self, state, action):
        #result(state, action) → state mới sau khi làm action
        raise NotImplementedError

    def step_cost(self, state, action):
        #step_cost(state, action) → chi phí (mặc định = 1)
        return 1

# =======================
# THUẬT TOÁN TÌM KIẾM
# =======================
#BFS dùng deque (hàng đợi), lấy theo FIFO. Nó sẽ mở rộng theo chiều rộng.
def bfs(problem: SearchProblem):
    frontier = deque([Node(problem.initial)])
    explored = set()
    while frontier:
        node = frontier.popleft()
        if problem.goal_test(node.state):
            return node.solution()
        explored.add(node.state)
        for action in problem.actions(node.state):
            child_state = problem.result(node.state, action)
            if child_state not in explored and all(n.state != child_state for n in frontier):
                frontier.append(Node(child_state, node, action, node.path_cost+1, node.depth+1))
    return None

#UCS dùng heapq (priority queue) , mọi bước đều cost = 1 → UCS chạy giống BFS.
def ucs(problem: SearchProblem):
    frontier = []
    heapq.heappush(frontier, (0, Node(problem.initial)))
    explored = {}
    while frontier:
        _, node = heapq.heappop(frontier)
        if problem.goal_test(node.state):
            return node.solution()
        if node.state in explored and explored[node.state] <= node.path_cost:
            continue
        explored[node.state] = node.path_cost
        for action in problem.actions(node.state):
            child_state = problem.result(node.state, action)
            cost = node.path_cost + problem.step_cost(node.state, action)
            child = Node(child_state, node, action, cost, node.depth+1)
            heapq.heappush(frontier, (child.path_cost, child))
    return None

#DFS dùng stack (list + pop), mở rộng theo chiều sâu.
def dfs(problem: SearchProblem, limit=None):
    frontier = [Node(problem.initial)]
    explored = set()
    while frontier:
        node = frontier.pop()
        if problem.goal_test(node.state):
            return node.solution()
        if limit is None or node.depth < limit:
            explored.add(node.state)
            for action in reversed(problem.actions(node.state)):
                child_state = problem.result(node.state, action)
                if child_state not in explored:
                    frontier.append(Node(child_state, node, action, node.path_cost+1, node.depth+1))
    return None

#Chỉ là DFS có limit về độ sâu
def dls(problem: SearchProblem, limit):
    return dfs(problem, limit)

#IDS = lặp DFS với limit = 0, 1, 2, ..., max_depth.
def ids(problem: SearchProblem, max_depth=50):
    for depth in range(max_depth+1):
        result = dfs(problem, limit=depth)
        if result is not None:
            return result
    return None


#bài 2: cài đặt lại sinh sâu đến trạng thái hello world test 5 hàm
goal_string = "HELLO WORLD"
problem = SearchProblem(initial="", goal=goal_string)

# Gán trực tiếp actions và result
def actions(state):
    if len(state) < len(problem.goal):
        return [problem.goal[len(state)]]
    return []

def result(state, action):
    return state + action

problem.actions = actions
problem.result = result

def run_with_time(func, *args, **kwargs):
    start = time.time()
    result = func(*args, **kwargs)
    end = time.time()
    return result, end - start

print("Goal:", goal_string)
print("="*40)

res, t = run_with_time(bfs, problem)
print("BFS result:", res)
print("Time:", t, "seconds")
print("="*40)

res, t = run_with_time(ucs, problem)
print("UCS result:", res)
print("Time:", t, "seconds")
print("="*40)

res, t = run_with_time(dfs, problem)
print("DFS result:", res)
print("Time:", t, "seconds")
print("="*40)

res, t = run_with_time(dls, problem, 11)
print("DLS result (limit=11):", res)
print("Time:", t, "seconds")
print("="*40)

res, t = run_with_time(ids, problem, max_depth=len(goal_string))
print("IDS result:", res)
print("Time:", t, "seconds")
print("="*40)

#bài 3: ô chữ 8 số,áp dụng 5 thuật toán , dùng mảng 2 chiều để hiện state
