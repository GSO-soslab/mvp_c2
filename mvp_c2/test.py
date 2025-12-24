node = TDMANode(node_id=0, role="master", tdma_cfg=cfg["tdma"], slots=[0])

while True:
    if node.should_send_sync():
        msg = node.master_sync_message()
        tx(msg)  # <-- your transport
    time.sleep(0.01)



node = TDMANode(node_id=1, role="slave", tdma_cfg=cfg["tdma"], slots=[1])

def on_sync(msg):
    node.handle_sync(msg)

while True:
    if node.in_my_slot():
        send_data()
    time.sleep(0.001)
