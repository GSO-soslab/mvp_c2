import heapq
import time

# only store the newest data in one group (dccl msg type).
# data will be removed if the total entry has exceeded or the data has expired.

class DynamicBufferPython:
    def __init__(self, max_total_size=100, drop_by_time= True):
        self._queue = []
        self.max_total_size = max_total_size
        self._groups = {} 
        self.drop_by_time = drop_by_time

    def push(self, data, priority, ttl_seconds, group_name=None):
        now = time.time()
        expiration = time.time() + ttl_seconds
        # We store [-priority, expiration, data, group_name, is_valid]
        # Using negative priority ensures the HIGHEST priority is at the top (index 0)
        entry = [-priority, expiration, now, data, group_name, [True]] 

        # 1. Handle Group Replacement
        is_update = False
        if group_name:
            if group_name in self._groups:
                # Mark old entry invalid
                self._groups[group_name][5][0] = False 
                is_update = True
            self._groups[group_name] = entry

        # 2. Handle Capacity (Drop oldest item if full and not just updating a group)
        if len(self._queue) >= self.max_total_size and not is_update:

            if self.drop_by_time:
                # Now index [2] correctly points to arrival_time
                oldest_idx = min(range(len(self._queue)), key=lambda i: self._queue[i][2])
                
                # Remove the oldest item to make room
                self._queue.pop(oldest_idx)
                heapq.heapify(self._queue)
            else:
                # Find the index of the LOWEST priority (the largest value at index [0])
                # In our case, Priority 1 is stored as -1, and Priority 10 is -10.
                # So the "max" of index [0] is the lowest priority number.
                lowest_priority_idx = max(range(len(self._queue)), key=lambda i: self._queue[i][0])
                
                # Optimization: If the NEW message is even lower priority than the worst one we have,
                # don't even bother adding it.
                if entry[0] > self._queue[lowest_priority_idx][0]:
                    return # Drop the incoming message
                    
                # Otherwise, remove the lowest and continue
                self._queue.pop(lowest_priority_idx)
                heapq.heapify(self._queue)

        # 3. Add to heap
        heapq.heappush(self._queue, entry)

    def pop(self):
        while self._queue:
            priority_neg, expiration, arrival, data, group, is_valid_ref = heapq.heappop(self._queue)
            
            # 1. Check if it was invalidated by a newer group member
            if not is_valid_ref[0]:
                continue
                
            # 2. Check if it has expired, lazy removal
            if time.time() > expiration:
                if group and self._groups.get(group)[3]== data:
                    del self._groups[group]
                continue # Discard expired item and move to next
                
            # 3. Success: Valid and not expired
            if group and group in self._groups:
                del self._groups[group]
            return data, group
            
        return None, None