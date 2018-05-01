import sys
import tensorflow as tf
from data_processing import *


tfnetwork=sys.argv[1]

def load_graph(tfnetwork):
	"""Creates a graph from saved GraphDef file and returns a saver."""
	# Creates graph from saved graph_def.pb.
	model_filename =tfnetwork
	with tf.gfile.GFile(model_filename, 'rb') as f:
		graph_def = tf.GraphDef()
		graph_def.ParseFromString(f.read())

	with tf.Graph().as_default() as graph:
		tf.import_graph_def(
			graph_def, 
			input_map=None, 
			return_elements=None, 
			name="", 
			op_dict=None, 
			producer_op_list=None
		)
	return graph



imagenetgraph=load_graph(tfnetwork)
#create_graph()
for op in imagenetgraph.get_operations():
	print(op.name)

featfile="data/train_position_feats.txt"
labelfile="data/train_position_labels.txt"
max_sequence_length=1000
sample_indx=0
BATCHSIZE=2
[features_train, labels_train, features_val, labels_val] = load_all_data(featfile, labelfile, max_sequence_length)
with tf.Session(graph=imagenetgraph) as sess:
	[features_train_batch, labels_train_batch] = prepare_input(sample_indx, BATCHSIZE, features_train, labels_train)
	input_tensor = sess.graph.get_tensor_by_name("input_features:0")
	predictions_tensor = sess.graph.get_tensor_by_name("prediction:0")
	print(features_train_batch.shape)
	predictions = sess.run(predictions_tensor,
				   {input_tensor: features_train_batch})
	print(predictions.shape)
