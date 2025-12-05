num_tasks, num_constraints = map(int, input().split())

constraints = []
for _ in range(num_constraints):
    i, j = map(int, input().split())
    constraints.append((i-1, j-1))

durations = {task: time for task, time in enumerate(list(map(int, input().split())))}
num_teams = int(input())
start_team_time = {team: time for team, time in enumerate(list(map(int, input().split())))}
num_costs = int(input())
costs = {}
for _ in range(num_costs):
    i, j, cost = map(int, input().split())
    costs[(i-1, j-1)] = cost

def find_cycles(edges):
    def dfs(v, visited, stack, path):
        visited[v] = True
        stack[v] = True
        path.append(v)
        
        for neighbor in graph.get(v, []):
            if stack.get(neighbor):  
                cycle_start_index = path.index(neighbor)
                cycles.append(path[cycle_start_index:])
            elif not visited.get(neighbor):
                dfs(neighbor, visited, stack, path)
        
        stack[v] = False
        path.pop()
    
    graph = {}
    for u, v in edges:
        if u not in graph:
            graph[u] = []
        graph[u].append(v)
    
    cycles = []
    visited = {}
    stack = {}
    
    for node in graph:
        visited[node] = False
        stack[node] = False

    for node in graph:
        if not visited[node]:
            dfs(node, visited, stack, [])
    
    return cycles

import itertools
cycles = set(itertools.chain(*find_cycles(constraints)))
remain_tasks = [task for task in range(num_tasks) if task not in cycles]


from ortools.sat.python import cp_model
MAX_INT = int(1e9)

# optimize the first objective
model = cp_model.CpModel()
assigned_tasks = {}
task2team = {}
start_task_time = {}

for task in remain_tasks:
    # assigned_tasks[task] = model.NewBoolVar(f"task{task}")
    start_task_time[task] = model.NewIntVar(0, MAX_INT, f"start{task}")

    for team in range(num_teams):
        if (task, team) in costs:
            task2team[(task, team)] = model.NewBoolVar(f"t2t{(task, team)}")

comp_time = model.NewIntVar(0, MAX_INT, f"max_time")

# constraints
for task in remain_tasks:
    model.Add(sum(task2team[(task, team)] for team in range(num_teams) \
                                                if (task, team) in costs) <= 1)
    model.Add(start_task_time[task] + durations[task] <= comp_time)

    for team in range(num_teams):
        if (task, team) in costs:
            model.Add(start_task_time[task] >= start_team_time[team] * task2team[(task, team)])

for task_i, task_j in constraints:
    if task_i in remain_tasks and task_j in remain_tasks:
        model.Add(start_task_time[task_j] >= start_task_time[task_i] + durations[task_i])
        
model.Maximize(sum(
    task2team[(task, team)]
    for task in remain_tasks
        for team in range(num_teams) if (task, team) in costs
))

solver = cp_model.CpSolver()
status = solver.Solve(model)

if status == cp_model.OPTIMAL or status == cp_model.FEASIBLE:
    max_tasks = int(solver.ObjectiveValue())
else:
    print("Can not find feasible solution!")
    exit()

# optimize the second objective
model = cp_model.CpModel()
assigned_tasks = {}
task2team = {}
start_task_time = {}

for task in remain_tasks:
    # assigned_tasks[task] = model.NewBoolVar(f"task{task}")
    start_task_time[task] = model.NewIntVar(0, MAX_INT, f"start{task}")

    for team in range(num_teams):
        if (task, team) in costs:
            task2team[(task, team)] = model.NewBoolVar(f"t2t{(task, team)}")

comp_time = model.NewIntVar(0, MAX_INT, f"max_time")

# constraints
for task in remain_tasks:
    model.Add(sum(task2team[(task, team)] for team in range(num_teams) \
                                                if (task, team) in costs) <= 1)
    model.Add(start_task_time[task] + durations[task] <= comp_time)

    for team in range(num_teams):
        if (task, team) in costs:
            model.Add(start_task_time[task] >= start_team_time[team] * task2team[(task, team)])

for task_i, task_j in constraints:
    if task_i in remain_tasks and task_j in remain_tasks:
        model.Add(start_task_time[task_j] >= start_task_time[task_i] + durations[task_i])

model.Add(sum(
    task2team[(task, team)]
    for task in remain_tasks
        for team in range(num_teams) if (task, team) in costs
) >= max_tasks)

model.Minimize(comp_time)

solver = cp_model.CpSolver()
status = solver.Solve(model)

if status == cp_model.OPTIMAL or status == cp_model.FEASIBLE:
    min_complete_time = int(solver.Value(comp_time))
else:
    print("Can not find feasible solution!")
    exit()

# optimize the third objective
model = cp_model.CpModel()
assigned_tasks = {}
task2team = {}
start_task_time = {}

for task in remain_tasks:
    # assigned_tasks[task] = model.NewBoolVar(f"task{task}")
    start_task_time[task] = model.NewIntVar(0, MAX_INT, f"start{task}")

    for team in range(num_teams):
        if (task, team) in costs:
            task2team[(task, team)] = model.NewBoolVar(f"t2t{(task, team)}")

comp_time = model.NewIntVar(0, MAX_INT, f"max_time")

# constraints
for task in remain_tasks:
    model.Add(sum(task2team[(task, team)] for team in range(num_teams) \
                                                if (task, team) in costs) <= 1)
    model.Add(start_task_time[task] + durations[task] <= min_complete_time)

    for team in range(num_teams):
        if (task, team) in costs:
            model.Add(start_task_time[task] >= start_team_time[team] * task2team[(task, team)])

for task_i, task_j in constraints:
    if task_i in remain_tasks and task_j in remain_tasks:
        model.Add(start_task_time[task_j] >= start_task_time[task_i] + durations[task_i])

model.Add(sum(
    task2team[(task, team)]
    for task in remain_tasks
        for team in range(num_teams) if (task, team) in costs
) >= max_tasks)

model.Minimize(sum(
    costs[(task, team)] * task2team[(task, team)]
    for task in remain_tasks
        for team in range(num_teams) if (task, team) in costs
))

solver = cp_model.CpSolver()
status = solver.Solve(model)

if status == cp_model.OPTIMAL or status == cp_model.FEASIBLE:
    print(max_tasks)

    for task in remain_tasks:
        for team in range(num_teams):
            if (task, team) in costs:
                
                if int(solver.Value(task2team[(task, team)])):
                    print(task + 1, team + 1, int(solver.Value(start_task_time[task])))
else:
    print("Can not find feasible solution!")
    exit()



        


