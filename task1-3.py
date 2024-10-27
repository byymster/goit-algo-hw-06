import networkx as nx
import matplotlib.pyplot as plt
from collections import deque
import heapq

metro_graph = nx.Graph()

stations = {
    "red": [
        "Академмістечко", "Житомирська", "Святошин", "Нивки", "Берестейська", "Шулявська",
        "Політехнічний інститут", "Вокзальна", "Університет", "Театральна", "Хрещатик",
        "Арсенальна", "Дніпро", "Гідропарк", "Лівобережна", "Дарниця", "Чернігівська", "Лісова"
    ],
    "blue": [
        "Героїв Дніпра", "Мінська", "Оболонь", "Почайна", "Тараса Шевченка",
        "Контрактова площа", "Поштова площа", "Майдан Незалежності", "Площа Українських героїв",
        "Олімпійська", "Палац Україна", "Либідська", "Деміївська", "Голосіївська",
        "Васильківська", "Виставковий центр", "Іподром", "Теремки"
    ],
    "green": [
        "Сирець", "Дорогожичі", "Лук'янівська", "Золоті ворота", "Палац спорту", "Кловська",
        "Печерська", "Звіринецька", "Видубичі", "Славутич", "Осокорки", "Позняки",
        "Харківська", "Вирлиця", "Бориспільська", "Червоний хутір"
    ]
}

for line_stations in stations.values():
    metro_graph.add_nodes_from(line_stations)

edges = {
    "red": [
        ("Академмістечко", "Житомирська", 3), ("Житомирська", "Святошин", 2),
        ("Святошин", "Нивки", 3), ("Нивки", "Берестейська", 2), ("Берестейська", "Шулявська", 2),
        ("Шулявська", "Політехнічний інститут", 3), ("Політехнічний інститут", "Вокзальна", 2),
        ("Вокзальна", "Університет", 3), ("Університет", "Театральна", 2),
        ("Театральна", "Хрещатик", 1), ("Хрещатик", "Арсенальна", 2),
        ("Арсенальна", "Дніпро", 2), ("Дніпро", "Гідропарк", 3),
        ("Гідропарк", "Лівобережна", 2), ("Лівобережна", "Дарниця", 3),
        ("Дарниця", "Чернігівська", 2), ("Чернігівська", "Лісова", 3)
    ],
    "blue": [
        ("Героїв Дніпра", "Мінська", 2), ("Мінська", "Оболонь", 3),
        ("Оболонь", "Почайна", 2), ("Почайна", "Тараса Шевченка", 2),
        ("Тараса Шевченка", "Контрактова площа", 3), ("Контрактова площа", "Поштова площа", 2),
        ("Поштова площа", "Майдан Незалежності", 2), ("Майдан Незалежності", "Площа Українських героїв", 1),
        ("Площа Українських героїв", "Олімпійська", 2), ("Олімпійська", "Палац Україна", 3),
        ("Палац Україна", "Либідська", 2), ("Либідська", "Деміївська", 3),
        ("Деміївська", "Голосіївська", 3), ("Голосіївська", "Васильківська", 2),
        ("Васильківська", "Виставковий центр", 3), ("Виставковий центр", "Іподром", 2),
        ("Іподром", "Теремки", 3)
    ],
    "green": [
        ("Сирець", "Дорогожичі", 3), ("Дорогожичі", "Лук'янівська", 2),
        ("Лук'янівська", "Золоті ворота", 3), ("Золоті ворота", "Палац спорту", 1),
        ("Палац спорту", "Кловська", 2), ("Кловська", "Печерська", 3),
        ("Печерська", "Звіринецька", 2), ("Звіринецька", "Видубичі", 3),
        ("Видубичі", "Славутич", 4), ("Славутич", "Осокорки", 2),
        ("Осокорки", "Позняки", 2), ("Позняки", "Харківська", 3),
        ("Харківська", "Вирлиця", 2), ("Вирлиця", "Бориспільська", 3),
        ("Бориспільська", "Червоний хутір", 2)
    ]
}

for line_edges in edges.values():
    metro_graph.add_weighted_edges_from(line_edges)

transfer_edges = [
    ("Майдан Незалежності", "Хрещатик", 5),
    ("Золоті ворота", "Театральна", 4),
    ("Палац спорту", "Площа Українських героїв", 3)
]
metro_graph.add_weighted_edges_from(transfer_edges)

# Алгоритм DFS для знаходження шляху
def dfs_path(graph, start, goal):
    stack = [(start, [start])]
    while stack:
        (vertex, path) = stack.pop()
        for neighbor in set(graph.neighbors(vertex)) - set(path):
            if neighbor == goal:
                yield path + [neighbor]
            else:
                stack.append((neighbor, path + [neighbor]))

# Алгоритм BFS для знаходження шляху
def bfs_path(graph, start, goal):
    queue = deque([(start, [start])])
    while queue:
        (vertex, path) = queue.popleft()
        for neighbor in set(graph.neighbors(vertex)) - set(path):
            if neighbor == goal:
                yield path + [neighbor]
            else:
                queue.append((neighbor, path + [neighbor]))

# Алгоритм Дейкстри для знаходження найкоротшого шляху
def dijkstra(graph, start):
    distances = {node: float('infinity') for node in graph.nodes}
    distances[start] = 0
    priority_queue = [(0, start)]
    while priority_queue:
        current_distance, current_node = heapq.heappop(priority_queue)
        if current_distance > distances[current_node]:
            continue
        for neighbor, attributes in graph[current_node].items():
            weight = attributes['weight']
            distance = current_distance + weight
            if distance < distances[neighbor]:
                distances[neighbor] = distance
                heapq.heappush(priority_queue, (distance, neighbor))
    return distances

# Приклад використання DFS і BFS
start_station = "Академмістечко"
goal_station = "Либідська"

print("Шляхи знайдені алгоритмом DFS:")
for path in dfs_path(metro_graph, start_station, goal_station):
    print(path)

print("\nШляхи знайдені алгоритмом BFS:")
for path in bfs_path(metro_graph, start_station, goal_station):
    print(path)

# Приклад використання алгоритму Дейкстри
print("\nНайкоротші шляхи від станції Академмістечко (алгоритм Дейкстри):")
distances = dijkstra(metro_graph, start_station)
for station, distance in distances.items():
    print(f"Відстань до {station}: {distance} хв")

pos = nx.kamada_kawai_layout(metro_graph, scale=3)
plt.figure(figsize=(14, 10))

colors = {"red": "red", "blue": "blue", "green": "green", "transfer": "purple"}
for line, line_edges in edges.items():
    nx.draw_networkx_edges(metro_graph, pos, edgelist=[(u, v) for u, v, w in line_edges], edge_color=colors[line], width=20)
nx.draw_networkx_edges(metro_graph, pos, edgelist=[(u, v) for u, v, w in transfer_edges], edge_color=colors["transfer"], style="dashed", width=2)

nx.draw_networkx_nodes(metro_graph, pos, node_size=200, node_color='lightblue')
nx.draw_networkx_labels(metro_graph, pos, font_size=10, font_weight='bold')

edge_labels = {(u, v): f"{w} хв" for u, v, w in metro_graph.edges(data="weight")}
nx.draw_networkx_edge_labels(metro_graph, pos, edge_labels=edge_labels, font_size=8)

plt.title("Мережа Київського метро з пересадками")
plt.axis("off")
plt.savefig("metro_graph.png")
plt.show()
