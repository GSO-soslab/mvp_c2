    #feature points
    def feature_points_callback(self, msg):
        proto = mvp_cmd_dccl_pb2.FeatureGeoPoints()
        #msg.data = [lat, lon, alt, lat, lon, alt]
        proto.time =round(time.time(), 3)
        proto.local_id = self.local_id
        proto.remote_id = self.remote_id
        proto.point_size = int(len(msg.data))

        for i in range(0, proto.point_size, 3):
            proto.latitude.append(msg.data[i]*100)
            proto.longitude.append(msg.data[i+1]*100)
            proto.altitude.append(msg.data[i+2]) 
        self.publish_dccl(proto)