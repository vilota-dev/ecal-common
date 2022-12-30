from ecal.core.publisher import MessagePublisher

class BytePublisher(MessagePublisher):
  """Spezialized publisher that sends out raw bytes
  """
  def __init__(self, name):
    topic_type = "base:byte"
    topic_desc = b""
    super(BytePublisher, self).__init__(name, topic_type, topic_desc)

  def send(self, msg, time=-1):
    return self.c_publisher.send(msg, time)