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

	int ToNodeId;	// 辺の終点頂点Id
	int Cost;		// 重み
};

// Pairのfirstとsecondの意味を忘れがちになるので今回はtypedefで意味を設定
typedef int ShortestDistance;
typedef int NodeId;
typedef std::pair<ShortestDistance, NodeId> Pair;	// first:最短距離 second:頂点番号

const int Infinity = 100000;	// 最短距離の最大値
const int NodeNum = 8;			// 頂点数

std::vector<Edge> g_Nodes[NodeNum];	// 頂点配列

void InitEdge()
{
	// (first, second) => (頂点番号、最短距離)
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
	int last_update_node_ids[NodeNum];											// 各頂点距離の最後に変更した頂点ID保存用
	std::fill(last_update_node_ids, last_update_node_ids + NodeNum, Infinity);	// 未測量状態として初期化

	int distances[NodeNum];									// 各頂点の最短距離保存用配列
	std::fill(distances, distances + NodeNum, Infinity);	// 未計測状態として初期化
	distances[start] = 0;									// 探索開始位置として初期化

	/*
		計測候補頂点保存用のキュー
			優先順位は最短距離が最も高く、同じ場合は頂点番号の小さい方とする

		優先順位付きキュー
			Pairを使った場合ソートはfirstを基準に行う
			firstが同じ値の場合はsecondでソートされる
	*/
	std::priority_queue<
		Pair,					// 要素はPair<int, int>
		std::vector<Pair>,		// コンテナの型 vectorを使用
		std::greater<Pair>		// 昇順でソートする(ソート方法の自作可能)
	> survey_node_que;

	// 測量候補頂点として指定された頂点を指定
	survey_node_que.push(Pair(0, start));

	while (survey_node_que.empty() == false)
	{
		/*
			計測候補キューの先頭から頂点を取得
			この頂点が始点、保存されてる各辺に登録されている
			頂点が終点として測量を行う
		*/
		Pair node = survey_node_que.top();
		// 先頭の削除
		survey_node_que.pop();

		ShortestDistance distance = node.first;
		NodeId node_id = node.second;

		/*
			計測対象の頂点までの最短距離(p.first)が
			その頂点に至るまでの最短距離(distances[node_id])よりも
			遠いなら次の頂点を探さない
		*/
		if (distances[node_id] < distance)
		{
			continue;
		}

		// 測量用頂点の辺の数だけ調べる
		for (int i = 0; i < g_Nodes[node_id].size(); i++)
		{
			Edge edge = g_Nodes[node_id][i];
			/*
				今の頂点までの距離 + 辺のコストが
				保存されている次の頂点までの最短距離よりも近ければ更新する
			*/
			if (distances[node_id] + edge.Cost < distances[edge.ToNodeId])
			{
				// 更新した経路の始点側の頂点IDを記録する
				last_update_node_ids[edge.ToNodeId] = node_id;

				// 最短距離を更新する
				distances[edge.ToNodeId] = distances[node_id] + edge.Cost;
				// 次の測量候補として登録する
				survey_node_que.push(Pair(distances[edge.ToNodeId], edge.ToNodeId));
			}
		}
	}

	// 最短ルートを取得
	int current_route_id = goal;		// 今チェック中の経路ID
	std::list<int> shortest_route;		// 経路保存用
	shortest_route.push_front(goal);

	while (true)
	{
		// 頂点配列から次の頂点IDを取得する
		int next_id = last_update_node_ids[current_route_id];

		// 始点じゃなかったら追加
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

	printf("最短ルートは");
	for (int id : shortest_route)
	{
		printf("%d ", id);
	}
	printf("\n");
	printf("最短距離は%d\n", distances[goal]);
}

int main()
{
	InitEdge();
	Dijkstra(0, 7);

	return 0;
}
