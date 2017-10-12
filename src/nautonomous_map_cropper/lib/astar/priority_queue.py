import heapq

# Priority Queue for astar searching
class PriorityQueue:
    def  __init__(self):
        self.heap = []
        self.count = 0

    # Push an element to the heap using a given priority
    def push(self, item, priority):
        entry = (priority, self.count, item)
        heapq.heappush(self.heap, entry)
        self.count += 1

    # Pop the element of the heap
    def pop(self):
        (_, _, item) = heapq.heappop(self.heap)
        return item

    # Check if the heap is empty.
    def isEmpty(self):
        return len(self.heap) == 0