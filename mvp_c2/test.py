def pop(self):
        now = time.time()
        while self._queue:
            # Pop the highest priority item
            priority_neg, expiration, data, group, is_valid_ref = heapq.heappop(self._queue)
            
            # 1. Skip if invalidated (replaced by a newer member of the same group)
            if not is_valid_ref[0]:
                continue
                
            # 2. Skip if expired
            if now > expiration:
                # Remove from group tracking if this was the latest entry
                if group and self._groups.get(group) is is_valid_ref:
                    del self._groups[group]
                continue 
                
            # 3. Success: Valid and not expired
            if group and group in self._groups:
                del self._groups[group]
            return data
            
        return None

    def cleanup_expired(self):
        """Optional: Call this to clear memory without extracting the top valid item."""
        now = time.time()
        # We can only efficiently remove expired items from the top of the heap.
        # Removing from the middle requires a full heap rebuild (O(N)).
        while self._queue and time.time() > self._queue[0][1]:
             # Peek at top: if expired, pop it
             item = heapq.heappop(self._queue)
             group = item[3]
             # Clean up group dict if necessary
             if group and self._groups.get(group) is item[4]:
                 del self._groups[group]