from collections import deque
import heapq
from typing import List, Tuple, Optional

# =======================
# Core data structures
# =======================
class Node:
    def __init__(self, state, parent=None, action=None, path_cost=0, depth=0):
        self.state = state          # state as a tuple of tuples (3x3)
        self.parent = parent        # parent Node
        self.action = action        # action that led here: 'Up','Down','Left','Right'
        self.path_cost = path_cost  # cumulative cost
        self.depth = depth          # depth in the tree

    def __lt__(self, other):
        # Required for heapq comparisons in UCS
        return self.path_cost < other.path_cost

    def solution(self):
        """Return (actions, states) along the path from root to this node."""
        actions, states = [], []
        node = self
        while node.parent is not None:
            actions.append(node.action)
            states.append(node.state)
            node = node.parent
        actions.reverse()
        states.reverse()
        return actions, states


class SearchProblem:
    def __init__(self, initial, goal):
        self.initial = initial
        self.goal = goal

    def goal_test(self, state):
        return state == self.goal

    def actions(self, state):
        raise NotImplementedError

    def result(self, state, action):
        raise NotImplementedError

    def step_cost(self, state, action):
        return 1


# =======================
# 8-Puzzle problem (state as 3x3 matrix)
# =======================
Matrix = Tuple[Tuple[int, int, int], Tuple[Tuple[int, int, int], ...]]

class EightPuzzleProblem(SearchProblem):
    """
    State format: tuple of 3 tuples (each length 3). Blank tile = 0.
    Example goal: ((1,2,3),(4,5,6),(7,8,0))
    """

    # Moves in fixed order for determinism
    MOVES = {
        'Up':    (-1, 0),
        'Down':  ( 1, 0),
        'Left':  ( 0,-1),
        'Right': ( 0, 1),
    }

    def __init__(self, initial: Tuple[Tuple[int, ...], ...], goal: Tuple[Tuple[int, ...], ...]):
        super().__init__(initial, goal)
#Duyệt ma trận 3x3 để tìm vị trí (row, col) của số 0 (ô trống). Trả về tọa độ để tính toán di chuyển.
    @staticmethod
    def _find_blank(state: Tuple[Tuple[int, ...], ...]):
        for i in range(3):
            for j in range(3):
                if state[i][j] == 0:
                    return i, j
        raise ValueError("No blank tile (0) in state")

    def actions(self, state: Tuple[Tuple[int, ...], ...]):
        i, j = self._find_blank(state)
        acts = []
        for name, (di, dj) in self.MOVES.items():
            ni, nj = i + di, j + dj
            if 0 <= ni < 3 and 0 <= nj < 3:
                acts.append(name)
        return acts

    def result(self, state: Tuple[Tuple[int, ...], ...], action: str):
        i, j = self._find_blank(state)
        di, dj = self.MOVES[action]
        ni, nj = i + di, j + dj
        # swap 0 with neighbor (ni, nj)
        new_grid = [list(row) for row in state]
        new_grid[i][j], new_grid[ni][nj] = new_grid[ni][nj], new_grid[i][j]
        return tuple(tuple(row) for row in new_grid)

    def step_cost(self, state, action):
        return 1  # unit step cost


# =======================
# Search algorithms
# =======================

def bfs(problem: SearchProblem):
    frontier = deque([Node(problem.initial)])
    visited = {problem.initial}
    while frontier:
        node = frontier.popleft()
        if problem.goal_test(node.state):
            return node.solution()
        for action in problem.actions(node.state):
            child_state = problem.result(node.state, action)
            if child_state not in visited:
                visited.add(child_state)
                frontier.append(Node(child_state, node, action, node.path_cost + problem.step_cost(node.state, action), node.depth + 1))
    return None


def ucs(problem: SearchProblem):
    frontier = []
    heapq.heappush(frontier, (0, Node(problem.initial)))
    best_cost = {problem.initial: 0}
    while frontier:
        _, node = heapq.heappop(frontier)
        if problem.goal_test(node.state):
            return node.solution()
        for action in problem.actions(node.state):
            child_state = problem.result(node.state, action)
            cost = node.path_cost + problem.step_cost(node.state, action)
            if child_state not in best_cost or cost < best_cost[child_state]:
                best_cost[child_state] = cost
                heapq.heappush(frontier, (cost, Node(child_state, node, action, cost, node.depth + 1)))
    return None


def dfs(problem: SearchProblem, limit: Optional[int] = None):
    stack = [Node(problem.initial)]
    visited = set()
    while stack:
        node = stack.pop()
        if problem.goal_test(node.state):
            return node.solution()
        if limit is not None and node.depth >= limit:
            continue
        if node.state in visited:
            continue
        visited.add(node.state)
        # Expand in reverse order to mimic typical DFS order control
        for action in reversed(problem.actions(node.state)):
            child_state = problem.result(node.state, action)
            if child_state not in visited:
                stack.append(Node(child_state, node, action, node.path_cost + problem.step_cost(node.state, action), node.depth + 1))
    return None


def dls(problem: SearchProblem, limit: int):
    # Recursive depth-limited search with path checking (avoids cycles along current path)
    def recursive(node: Node, depth: int):
        if problem.goal_test(node.state):
            return node.solution()
        if depth == limit:
            return 'cutoff'
        cutoff = False
        for action in problem.actions(node.state):
            child_state = problem.result(node.state, action)
            # avoid returning to an ancestor state (cycle). Walk up parents.
            anc = node
            cycle = False
            while anc is not None:
                if anc.state == child_state:
                    cycle = True
                    break
                anc = anc.parent
            if cycle:
                continue
            child = Node(child_state, node, action, node.path_cost + problem.step_cost(node.state, action), node.depth + 1)
            result = recursive(child, depth + 1)
            if result == 'cutoff':
                cutoff = True
            elif result is not None:
                return result
        return 'cutoff' if cutoff else None

    res = recursive(Node(problem.initial), 0)
    return None if res == 'cutoff' else res


def ids(problem: SearchProblem, max_depth: int = 50):
    for depth in range(max_depth + 1):
        res = dls(problem, depth)
        if res is not None:
            return res
    return None


# =======================
# Utilities
# =======================

def pretty_print(state: Tuple[Tuple[int, ...], ...]):
    for row in state:
        print(' '.join('_' if x == 0 else str(x) for x in row))
    print()


def print_path(states: List[Tuple[Tuple[int, ...], ...]]):
    for idx, s in enumerate(states):
        print(f"Step {idx + 1}:")
        pretty_print(s)


def is_solvable(state: Tuple[Tuple[int, ...], ...]) -> bool:
    # Flatten without the blank
    flat = [x for row in state for x in row if x != 0]
    inv = 0
    for i in range(len(flat)):
        for j in range(i + 1, len(flat)):
            if flat[i] > flat[j]:
                inv += 1
    # For 3x3, solvable iff inversion count is even
    return inv % 2 == 0


# =======================
# Demo run (no __main__ guard, runs immediately)
# =======================
# Start and goal as matrices (tuples of tuples)
start = (
    (1, 2, 3),
    (4, 0, 6),
    (7, 5, 8),
)

goal = (
    (1, 2, 3),
    (4, 5, 6),
    (7, 8, 0),
)

print("Trạng thái bắt đầu:")
pretty_print(start)
print("Trạng thái đích:")
pretty_print(goal)

if not is_solvable(start):
    print("⚠️ Trạng thái bắt đầu KHÔNG giải được cho bài toán 8-puzzle 3x3 (tổng số nghịch thế lẻ).")
else:
    problem = EightPuzzleProblem(initial=start, goal=goal)

    print("=== Thuật toán BFS ===")
    sol = bfs(problem)
    print("Dãy hành động:", sol[0] if sol else None, "\n")

    print("=== Thuật toán UCS ===")
    sol = ucs(problem)
    print("Dãy hành động:", sol[0] if sol else None, "\n")

    print("=== Thuật toán DFS ===")
    sol = dfs(problem)
    if sol:
        actions, states = sol
        print(f"Tìm thấy lời giải với {len(actions)} bước di chuyển")
    else:
        print("Không tìm thấy lời giải")
    print()

    print("=== Thuật toán DLS (giới hạn độ sâu = 12) ===")
    sol = dls(problem, limit=12)
    if sol:
        actions, states = sol
        print(f"Tìm thấy trong giới hạn độ sâu 12, số bước = {len(actions)}")
    else:
        print("Không tìm thấy lời giải trong giới hạn")
    print()

    print("=== Thuật toán IDS (độ sâu tối đa = 25) ===")
    sol = ids(problem, max_depth=25)
    if sol:
        actions, states = sol
        print(f"Tìm thấy lời giải với {len(actions)} bước di chuyển")
    else:
        print("Không tìm thấy lời giải trong giới hạn")
    print()

    # Nếu muốn in chi tiết đường đi (ví dụ BFS):
    # if sol:
    #     actions, states = sol
    #     print("Chuỗi hành động:", actions)
    #     print_path(states)

