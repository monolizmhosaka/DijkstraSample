#include <stdio.h>
#include <utility>
#include <queue>
#include <vector>
#include <list>

struct Edge
{
	Edge(int v, int c)
	{
		ToNodeId = v;
		Cost = c;
	}

	int ToNodeId;	// �ӂ̏I�_���_Id
	int Cost;		// �d��
};

// Pair��first��second�̈Ӗ���Y�ꂪ���ɂȂ�̂ō����typedef�ňӖ���ݒ�
typedef int ShortestDistance;
typedef int NodeId;
typedef std::pair<ShortestDistance, NodeId> Pair;	// first:�ŒZ���� second:���_�ԍ�

const int Infinity = 100000;	// �ŒZ�����̍ő�l
const int NodeNum = 8;			// ���_��

std::vector<Edge> g_Nodes[NodeNum];	// ���_�z��

void InitEdge()
{
	// (first, second) => (���_�ԍ��A�ŒZ����)
	g_Nodes[0].emplace_back(2, 1);
	g_Nodes[0].emplace_back(3, 4);
	g_Nodes[0].emplace_back(4, 5);

	g_Nodes[1].emplace_back(5, 4);
	g_Nodes[1].emplace_back(7, 8);

	g_Nodes[2].emplace_back(1, 1);

	g_Nodes[3].emplace_back(6, 4);

	g_Nodes[4].emplace_back(5, 2);
	g_Nodes[4].emplace_back(6, 2);

	g_Nodes[5].emplace_back(7, 2);

	g_Nodes[6].emplace_back(7, 5);
}

void Dijkstra(int start, int goal)
{
	int last_update_node_ids[NodeNum];											// �e���_�����̍Ō�ɕύX�������_ID�ۑ��p
	std::fill(last_update_node_ids, last_update_node_ids + NodeNum, Infinity);	// �����ʏ�ԂƂ��ď�����

	int distances[NodeNum];									// �e���_�̍ŒZ�����ۑ��p�z��
	std::fill(distances, distances + NodeNum, Infinity);	// ���v����ԂƂ��ď�����
	distances[start] = 0;									// �T���J�n�ʒu�Ƃ��ď�����

	/*
		�v����Ⓒ�_�ۑ��p�̃L���[
			�D�揇�ʂ͍ŒZ�������ł������A�����ꍇ�͒��_�ԍ��̏��������Ƃ���

		�D�揇�ʕt���L���[
			Pair���g�����ꍇ�\�[�g��first����ɍs��
			first�������l�̏ꍇ��second�Ń\�[�g�����
	*/
	std::priority_queue<
		Pair,					// �v�f��Pair<int, int>
		std::vector<Pair>,		// �R���e�i�̌^ vector���g�p
		std::greater<Pair>		// �����Ń\�[�g����(�\�[�g���@�̎���\)
	> survey_node_que;

	// ���ʌ�Ⓒ�_�Ƃ��Ďw�肳�ꂽ���_���w��
	survey_node_que.push(Pair(0, start));

	while (survey_node_que.empty() == false)
	{
		/*
			�v�����L���[�̐擪���璸�_���擾
			���̒��_���n�_�A�ۑ�����Ă�e�ӂɓo�^����Ă���
			���_���I�_�Ƃ��đ��ʂ��s��
		*/
		Pair node = survey_node_que.top();
		// �擪�̍폜
		survey_node_que.pop();

		ShortestDistance distance = node.first;
		NodeId node_id = node.second;

		/*
			�v���Ώۂ̒��_�܂ł̍ŒZ����(p.first)��
			���̒��_�Ɏ���܂ł̍ŒZ����(distances[node_id])����
			�����Ȃ玟�̒��_��T���Ȃ�
		*/
		if (distances[node_id] < distance)
		{
			continue;
		}

		// ���ʗp���_�̕ӂ̐��������ׂ�
		for (int i = 0; i < g_Nodes[node_id].size(); i++)
		{
			Edge edge = g_Nodes[node_id][i];
			/*
				���̒��_�܂ł̋��� + �ӂ̃R�X�g��
				�ۑ�����Ă��鎟�̒��_�܂ł̍ŒZ���������߂���΍X�V����
			*/
			if (distances[node_id] + edge.Cost < distances[edge.ToNodeId])
			{
				// �X�V�����o�H�̎n�_���̒��_ID���L�^����
				last_update_node_ids[edge.ToNodeId] = node_id;

				// �ŒZ�������X�V����
				distances[edge.ToNodeId] = distances[node_id] + edge.Cost;
				// ���̑��ʌ��Ƃ��ēo�^����
				survey_node_que.push(Pair(distances[edge.ToNodeId], edge.ToNodeId));
			}
		}
	}

	// �ŒZ���[�g���擾
	int current_route_id = goal;		// ���`�F�b�N���̌o�HID
	std::list<int> shortest_route;		// �o�H�ۑ��p
	shortest_route.push_front(goal);

	while (true)
	{
		// ���_�z�񂩂玟�̒��_ID���擾����
		int next_id = last_update_node_ids[current_route_id];

		// �n�_����Ȃ�������ǉ�
		if (current_route_id != start)
		{
			shortest_route.push_front(next_id);
			current_route_id = next_id;
		}
		else
		{
			break;
		}
	}

	printf("�ŒZ���[�g��");
	for (int id : shortest_route)
	{
		printf("%d ", id);
	}
	printf("\n");
	printf("�ŒZ������%d\n", distances[goal]);
}

int main()
{
	InitEdge();
	Dijkstra(0, 7);

	return 0;
}
