import heapq
import time

class DynamicBufferPython:
    def __init__(self, max_total_size=100):
        self._queue = []
        self.max_total_size = max_total_size
        self._groups = {} 

    def push(self, data, priority, ttl_seconds, group_name=None):
        expiration = time.time() + ttl_seconds
        # We store [-priority, expiration, data, group_name, is_valid]
        # Using negative priority ensures the HIGHEST priority is at the top (index 0)
        entry = [-priority, expiration, data, group_name, [True]] 

        # 1. Handle Group Replacement
        if group_name:
            if group_name in self._groups:
                # Mark old entry invalid. We use a list [True] so it's a mutable reference
                self._groups[group_name][4][0] = False 
            self._groups[group_name] = entry

        # 2. Handle Capacity (Drop lowest priority)
        if len(self._queue) >= self.max_total_size:
            # We need to find the LOWEST priority (the largest negative number)
            # This is slow in a min-heap (O(n)). 
            # Optimization: Sort or use a secondary structure if max_size is very large.
            # For max_total_size=100, we can find the max index:
            lowest_priority_idx = max(range(len(self._queue)), key=lambda i: self._queue[i][0])
            
            # If the new item is actually lower priority than the lowest, don't add it
            if entry[0] > self._queue[lowest_priority_idx][0]:
                return # Drop the incoming message
            
            # Otherwise, remove the lowest and add the new one
            self._queue.pop(lowest_priority_idx)
            heapq.heapify(self._queue)

        heapq.heappush(self._queue, entry)

    def pop(self):
        while self._queue:
            priority_neg, expiration, data, group, is_valid_ref = heapq.heappop(self._queue)
            
            # 1. Check if it was invalidated by a newer group member
            if not is_valid_ref[0]:
                continue
                
            # 2. Check if it has expired
            if time.time() > expiration:
                if group and self._groups.get(group)[2] == data:
                    del self._groups[group]
                continue # Discard expired item and move to next
                
            # 3. Success: Valid and not expired
            if group and group in self._groups:
                del self._groups[group]
            return data
            
        return None