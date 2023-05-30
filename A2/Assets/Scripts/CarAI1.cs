using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using System.Linq;

namespace UnityStandardAssets.Vehicles.Car
{
    [RequireComponent(typeof(CarController))]
    public class CarAI1 : MonoBehaviour
    {
        private CarController m_Car; // the car controller we want to use
        private Rigidbody m_Car_rb;
        public GameObject terrain_manager_game_object;
        TerrainManager terrain_manager;

        public GameObject[] friends;
        public GameObject[] enemies;
        
        private Dictionary<Vector3, HashSet<Vector3>> adjacencyList = new Dictionary<Vector3, HashSet<Vector3>>(); // Graph G = (V, E) where V is a set of vertices and E is a set of edges
        public Dictionary<Vector3, HashSet<Vector3>> spanningTree = new Dictionary<Vector3, HashSet<Vector3>>();
        public Dictionary<Vector3, HashSet<Vector3>> remainingSpanningTree = new Dictionary<Vector3, HashSet<Vector3>>();
        public HashSet<Vector3> remainingTreeNodes = new HashSet<Vector3>();

        private Vector3[] nearestNodes;
        private Vector3[] start_pos;

        public Vector3 root;
        public Node root1, root2, root3;
        public Dictionary<Vector3, Vector3> path_edges, path1_edges, path2_edges, path3_edges;
        public List<Vector3> pathway_points, pathway_points1, pathway_points2, pathway_points3;
        
        private float start_time;

        private bool isTraversable(float[,] traversability, int row, int col) {
            return traversability[row, col] < 0.5f;
        }

        private bool areAdjacentCells(Vector3 c1, Vector3 c2) {
            return (c1.x == c2.x && Math.Abs(c1.z - c2.z) == 20) || (c1.z == c2.z && Math.Abs(c1.x - c2.x) == 20);
        }

        private Vector3 getCenterOfCell(int row, int col, float xStep, float zStep) {
            return new Vector3(terrain_manager.myInfo.x_low + (row - 1 + 0.5f) * xStep, terrain_manager.myInfo.start_pos.y, terrain_manager.myInfo.z_low + (col - 1 + 0.5f) * zStep);
        }

        private List<Vector3> getValidCenters() {
            float[,] traversabilityMatrix = terrain_manager.myInfo.GetPaddedTraversability();
            float xStep = (terrain_manager.myInfo.x_high - terrain_manager.myInfo.x_low) / terrain_manager.myInfo.x_N;
            float zStep = (terrain_manager.myInfo.z_high - terrain_manager.myInfo.z_low) / terrain_manager.myInfo.z_N;
            int rows = traversabilityMatrix.GetLength(0);
            int columns = traversabilityMatrix.GetLength(1);
            List<Vector3> validCenters = new List<Vector3>();

            for (int r = 1; r < rows - 1; r++) {
                for (int c = 1; c < columns - 1; c++) {
                    if (isTraversable(traversabilityMatrix, r, c)) { 
                        Vector3 center = getCenterOfCell(r, c, xStep, zStep);
                        validCenters.Add(center);
                    }
                }
            }
            return validCenters;
        }

        private Dictionary<Vector3, HashSet<Vector3>> getAdjacencyList(List<Vector3> validCenters) {
            Dictionary<Vector3, HashSet<Vector3>> adjacencyList = new Dictionary<Vector3, HashSet<Vector3>>();
            foreach (Vector3 center in validCenters) {
                adjacencyList.Add(center, new HashSet<Vector3>());

                foreach (Vector3 center1 in adjacencyList.Keys) {
                    if (areAdjacentCells(center1, center)) {
                        adjacencyList[center].Add(center1);
                        adjacencyList[center1].Add(center);
                    }
                }
            }
            return adjacencyList;
        }

        public Dictionary<Vector3, HashSet<Vector3>> getSpanningTree()
        {
            Dictionary<Vector3, HashSet<Vector3>> spanningTree = new Dictionary<Vector3, HashSet<Vector3>>();
            HashSet<(Vector3 from, Vector3 to)> edges = new HashSet<(Vector3 from, Vector3 to)>();
            foreach (Vector3 cell in adjacencyList.Keys)
            {
                foreach (Vector3 neighbor in adjacencyList[cell])
                {
                    edges.Add((cell, neighbor));
                }
            }

            Dictionary<Vector3, Vector3> parents = new Dictionary<Vector3, Vector3>();
            foreach (Vector3 cell in adjacencyList.Keys)
            {
                parents[cell] = cell;
            }
            foreach ((Vector3 from, Vector3 to) edge in edges)
            {
                Vector3 parent_from_cell1 = find(parents, edge.from);
                Vector3 parent_to_cell2 = find(parents, edge.to);

                // Check if the edge creates a cycle in the spanning tree
                if (parent_from_cell1 != parent_to_cell2)
                {
                    if (!spanningTree.ContainsKey(edge.from))
                    {
                        spanningTree[edge.from] = new HashSet<Vector3>();
                    }
                    spanningTree[edge.from].Add(edge.to);

                    if (!spanningTree.ContainsKey(edge.to))
                    {
                        spanningTree[edge.to] = new HashSet<Vector3>();
                    }
                    spanningTree[edge.to].Add(edge.from);
                    parents[parent_from_cell1] = parent_to_cell2;
                }
            }
            return spanningTree;
        }

        // Helper function to find the parent of a cell in the spanning tree
        // to check if adding an edge will create a cycle in the tree.
        private Vector3 find(Dictionary<Vector3, Vector3> parents, Vector3 cell)
        {
            if (parents[cell] == cell) // If the cell is its own parent, it is the root of the tree
            {
                return cell;
            }
            else // Otherwise, find the root of the tree
            {
                return find(parents, parents[cell]);
            }
        }

       
        private void Start()
        {
            // get the car controller
            m_Car = GetComponent<CarController>();
            m_Car_rb = GetComponent<Rigidbody>();
            terrain_manager = terrain_manager_game_object.GetComponent<TerrainManager>();
            // note that both arrays will have holes when objects are destroyed
            // but for initial planning they should work
            friends = GameObject.FindGameObjectsWithTag("Player");
            // Note that you are not allowed to check the positions of the turrets in this problem

            start_pos = new Vector3[friends.Length];
            Debug.Log("friends number: " + friends.Length);
            for(int i = 0; i < friends.Length; i++)
            {
                start_pos[i] = friends[i].transform.position;
                Debug.Log("start_pos[" + i + "]: " + start_pos[i]);
            }

            // Plan your path here
            adjacencyList = getAdjacencyList(getValidCenters());
            remainingSpanningTree = spanningTree = getSpanningTree();

            
            foreach (Vector3 pos in spanningTree.Keys)
            {
                Debug.Log("pos: " + pos);
                Debug.Log("num: " + spanningTree[pos].Count);
                foreach (Vector3 adj in spanningTree[pos])
                {
                    Debug.Log("pos:" + pos + "-> adj:" + adj);
                    if (adj.x == 220f && adj.z == 240f)
                    {
                        root = adj;
                        Debug.Log("root: " + root);
                    }
                    Debug.DrawLine(pos, adj, Color.yellow, 2.5f);
                }
            }
            // Debug.DrawLine(new Vector3(220f, 0f, 240f), new Vector3(221f, 1f, 241f), Color.red, 10f);

            // Debug.DrawLine(new Vector3(225f, 0f, 235f), new Vector3(228f, 0f, 238f), Color.blue, 50f);
            Tree tree = new Tree(spanningTree, root);
            Tree wholetree = new Tree(spanningTree, root);
            
            tree.CountSubtreeNodes(tree.root);
            tree.PrintTree();

            int subtreeNodesNum = Mathf.CeilToInt(tree.nodesNum / 3.0f);
            Debug.Log("subtreeNodesNum: " + subtreeNodesNum);
            tree.visitNode(tree.root, subtreeNodesNum);
            Debug.Log("root2: " + tree.root2.pos);
            Debug.Log("root3: " + tree.root3.pos);
            Debug.Log("root2 adj count: " + spanningTree[tree.root2.parent.pos].Count);
            Debug.Log("root3 adj count: " + spanningTree[tree.root3.parent.pos].Count);
            spanningTree[tree.root2.pos].Remove(tree.root2.parent.pos);
            spanningTree[tree.root2.parent.pos].Remove(tree.root2.pos);
            spanningTree[tree.root3.pos].Remove(tree.root3.parent.pos);
            spanningTree[tree.root3.parent.pos].Remove(tree.root3.pos);

            Debug.Log("root2 adj count new: " + spanningTree[tree.root2.parent.pos].Count);
            Debug.Log("root3 adj count new: " + spanningTree[tree.root3.parent.pos].Count);

            if (Vector3.Distance(tree.root2.parent.left.pos, tree.root2.pos) == 0f) tree.root2.parent.left = null;
            else if (Vector3.Distance(tree.root2.parent.right.pos, tree.root2.pos) == 0f) tree.root2.parent.right = null;
            
            if (Vector3.Distance(tree.root3.parent.left.pos, tree.root3.pos) == 0f) tree.root3.parent.left = null;
            else if (Vector3.Distance(tree.root3.parent.right.pos, tree.root3.pos) == 0f) tree.root3.parent.right = null;
            tree.root2.parent = null;
            tree.root3.parent = null;
            // Debug.DrawLine(tree.root1.pos, new Vector3(tree.root1.pos.x + 5, 0f, tree.root1.pos.z + 5), Color.blue, 50f);
            // Debug.DrawLine(tree.root2.pos, new Vector3(tree.root2.pos.x + 5, 0f, tree.root2.pos.z + 5), Color.blue, 50f);
            // Debug.DrawLine(tree.root3.pos, new Vector3(tree.root3.pos.x + 5, 0f, tree.root3.pos.z + 5), Color.blue, 50f);


            path1_edges = new Dictionary<Vector3, Vector3>();
            path2_edges = new Dictionary<Vector3, Vector3>();
            path3_edges = new Dictionary<Vector3, Vector3>();
            GetPathAroundNode(tree.root1, path1_edges);
            GetPathAroundNode(tree.root2, path2_edges);
            GetPathAroundNode(tree.root3, path3_edges);
            GetPathBetweenNodes(tree.root1, tree.root1.left, path1_edges);
            GetPathBetweenNodes(tree.root2, tree.root2.left, path2_edges);
            GetPathBetweenNodes(tree.root3, tree.root3.left, path3_edges);
            GetPathBetweenNodes(tree.root1, tree.root1.right, path1_edges);
            GetPathBetweenNodes(tree.root2, tree.root2.right, path2_edges);
            GetPathBetweenNodes(tree.root3, tree.root3.right, path3_edges);
            foreach (Vector3 from in path1_edges.Keys)
            {
                Vector3 to = path1_edges[from];
                Debug.DrawLine(from, to, Color.red, 2f);
            }
            foreach (Vector3 from in path2_edges.Keys)
            {
                Vector3 to = path2_edges[from];
                Debug.DrawLine(from, to, Color.blue, 2f);
            }
            foreach (Vector3 from in path3_edges.Keys)
            {
                Vector3 to = path3_edges[from];
                Debug.DrawLine(from, to, Color.black, 2f);
            }


            Vector3 startpoint1 = FindStartPoint(tree.root1, path1_edges);
            Vector3 startpoint2 = FindStartPoint(tree.root2, path2_edges);
            Vector3 startpoint3 = FindStartPoint(tree.root3, path3_edges);

            //Debug.DrawLine(startpoint1, new Vector3(startpoint1.x + 5, 0f, startpoint1.z + 5), Color.blue, 50f);
            //Debug.DrawLine(startpoint2, new Vector3(startpoint2.x + 5, 0f, startpoint2.z + 5), Color.blue, 50f);
            //Debug.DrawLine(startpoint3, new Vector3(startpoint3.x + 5, 0f, startpoint3.z + 5), Color.blue, 50f);


            pathway_points1 = new List<Vector3>();
            pathway_points2 = new List<Vector3>();
            pathway_points3 = new List<Vector3>();
            FindPathWayBeforeStartPoint(wholetree, tree.root1, pathway_points1);
            FindPathWayBeforeStartPoint(wholetree, tree.root2, pathway_points2);
            FindPathWayBeforeStartPoint(wholetree, tree.root3, pathway_points3);

            FindPathWay(startpoint1, path1_edges, pathway_points1);
            FindPathWay(startpoint2, path2_edges, pathway_points2);
            FindPathWay(startpoint3, path3_edges, pathway_points3);
            Debug.Log("path1 num: " + pathway_points1.Count);
            Debug.Log("path2 num: " + pathway_points2.Count);
            Debug.Log("path3 num: " + pathway_points3.Count);
            start_time = Time.time;
        }
        
        public void FindPathWayBeforeStartPoint(Tree tree, Node root, List<Vector3> pathway_points)
        {
            Node node = tree.root;
            while (node != null && node.pos != root.pos)
            {
                Debug.Log("node.pos: " + node.pos);
                pathway_points.Add(node.pos);
                float dis1 = 0;
                float dis2 = 0;
                if (node.left != null && node.right != null)
                {
                    dis1 = Vector3.Distance(node.left.pos, root.pos);
                    dis2 = Vector3.Distance(node.right.pos, root.pos);
                    if (dis1 <= dis2) node = node.left;
                    else if (dis1 > dis2) node = node.right;
                }
                else if (node.left == null && node.right != null)
                {
                    node = node.right;
                }
                else if (node.right == null && node.left != null)
                {
                    node = node.left;
                }
                else
                {
                    node = null;
                }
            }
        }

        public void FindPathWay(Vector3 startpoint, Dictionary<Vector3, Vector3> path_edges, List<Vector3> pathway_points)
        {
            Vector3 next_point = startpoint;
            Vector3 final_point = startpoint; 
            pathway_points.Add(next_point);
            next_point = path_edges[next_point];
            while (next_point != final_point)
            {
                // Debug.Log("next_point:" + next_point);
                pathway_points.Add(next_point);
                next_point = path_edges[next_point];
            }
            
        }

        public Vector3 FindStartPoint(Node node, Dictionary<Vector3, Vector3> path_edges)
        {
            Vector3 topleft = new Vector3(node.pos.x - 5, 0f, node.pos.z + 5);
            Vector3 topright = new Vector3(node.pos.x + 5, 0f, node.pos.z + 5);
            Vector3 bottomleft = new Vector3(node.pos.x - 5, 0f, node.pos.z - 5);
            Vector3 bottomright = new Vector3(node.pos.x + 5, 0f, node.pos.z - 5);
            if (path_edges.ContainsKey(topleft))
            {
                return topleft;
            }
            else if (path_edges.ContainsKey(topright))
            {
                return topright;
            }
            else if (path_edges.ContainsKey(bottomleft))
            {
                return bottomleft;
            }
            else if (path_edges.ContainsKey(bottomright))
            {
                return bottomright;
            }
            return Vector3.zero;
        }
        

        public void GetPathBetweenNodes(Node node1, Node node2, Dictionary<Vector3, Vector3> path_edges)
        {
            if (node2 == null) return;
            // node2 <= node1
            if (node1.pos.x > node2.pos.x)
            {
                Vector3 node1_topleft = new Vector3(node1.pos.x - 5, 0f, node1.pos.z + 5);
                Vector3 node1_bottomleft = new Vector3(node1.pos.x - 5, 0f, node1.pos.z - 5);
                Vector3 node2_topright = new Vector3(node2.pos.x + 5, 0f, node2.pos.z + 5);
                Vector3 node2_bottomright = new Vector3(node2.pos.x + 5, 0f, node2.pos.z - 5);
                path_edges[node1_topleft] = node2_topright;
                path_edges[node2_bottomright] = node1_bottomleft;
            }
            // node1 => node2
            if (node1.pos.x < node2.pos.x)
            {
                Vector3 node2_topleft = new Vector3(node2.pos.x - 5, 0f, node2.pos.z + 5);
                Vector3 node2_bottomleft = new Vector3(node2.pos.x - 5, 0f, node2.pos.z - 5);
                Vector3 node1_topright = new Vector3(node1.pos.x + 5, 0f, node1.pos.z + 5);
                Vector3 node1_bottomright = new Vector3(node1.pos.x + 5, 0f, node1.pos.z - 5);
                path_edges[node2_topleft] = node1_topright;
                path_edges[node1_bottomright] = node2_bottomleft;
            }
            // node2
            //  ||
            // node1
            if (node1.pos.z < node2.pos.z)
            {
                Vector3 node1_topleft = new Vector3(node1.pos.x - 5, 0f, node1.pos.z + 5);
                Vector3 node1_topright = new Vector3(node1.pos.x + 5, 0f, node1.pos.z + 5);
                Vector3 node2_bottomleft = new Vector3(node2.pos.x - 5, 0f, node2.pos.z - 5);
                Vector3 node2_bottomright = new Vector3(node2.pos.x + 5, 0f, node2.pos.z - 5);
                path_edges[node2_bottomleft] = node1_topleft;
                path_edges[node1_topright] = node2_bottomright;
            }
            // node1
            //  ||
            // node2
            if (node1.pos.z > node2.pos.z)
            {
                Vector3 node2_topleft = new Vector3(node2.pos.x - 5, 0f, node2.pos.z + 5);
                Vector3 node2_topright = new Vector3(node2.pos.x + 5, 0f, node2.pos.z + 5);
                Vector3 node1_bottomleft = new Vector3(node1.pos.x - 5, 0f, node1.pos.z - 5);
                Vector3 node1_bottomright = new Vector3(node1.pos.x + 5, 0f, node1.pos.z - 5);
                path_edges[node1_bottomleft] = node2_topleft;
                path_edges[node2_topright] = node1_bottomright;
            }
            GetPathBetweenNodes(node2, node2.left, path_edges);
            GetPathBetweenNodes(node2, node2.right, path_edges);
            GetPathBetweenNodes(node2, node2.third, path_edges);
        }

        public void GetPathAroundNode(Node node, Dictionary<Vector3, Vector3> path_edges)
        {
            if (node == null) return;
            Vector3 topleft = new Vector3(node.pos.x - 5, 0f, node.pos.z + 5);
            Vector3 topright = new Vector3(node.pos.x + 5, 0f, node.pos.z + 5);
            Vector3 bottomleft = new Vector3(node.pos.x - 5, 0f, node.pos.z - 5);
            Vector3 bottomright = new Vector3(node.pos.x + 5, 0f, node.pos.z - 5);
            path_edges[topright] = topleft;
            path_edges[topleft] = bottomleft;
            path_edges[bottomleft] = bottomright;
            path_edges[bottomright] = topright;
            
            Debug.Log("getallpath num:" + spanningTree[node.pos].Count);
            foreach (Vector3 adj in spanningTree[node.pos])
            {
                if (adj.x > node.pos.x) path_edges.Remove(bottomright);
                if (adj.x < node.pos.x) path_edges.Remove(topleft);
                if (adj.z > node.pos.z) path_edges.Remove(topright);
                if (adj.z < node.pos.z) path_edges.Remove(bottomleft);
            }
            
            GetPathAroundNode(node.left, path_edges);
            GetPathAroundNode(node.right, path_edges);
            GetPathAroundNode(node.third, path_edges);
        }


        public class Node
        {
            public Vector3 pos;
            public Node parent;
            public Node left;
            public Node right;
            public Node third;
            public int childrenCount = 0;
            public Node(Vector3 pos, Node parent = null)
            {
                this.pos = pos;
                this.parent = parent;
            }
        }

        public class Tree
        {
            private Dictionary<Vector3, HashSet<Vector3>> spanningTree;
            public Node root;
            public Node root1, root2, root3;
            public int nodesNum = 0;
            public Tree(Dictionary<Vector3, HashSet<Vector3>> spanningTree, Vector3 root)
            {
                this.spanningTree = spanningTree;
                this.root = new Node(root);
                this.root1 = this.root;
                BuildTree(this.root, null);
            }
            private void BuildTree(Node node, Node pre)
            {
                HashSet<Vector3> childrenPositions;
                nodesNum++;
                if (spanningTree.TryGetValue(node.pos, out childrenPositions))
                {
                    if (pre != null) childrenPositions.Remove(pre.pos);
                    // Build the left child of the current node
                    Vector3 leftChildPosition = childrenPositions.FirstOrDefault();
                    if (leftChildPosition != Vector3.zero)
                    {
                        Node leftChild = new Node(leftChildPosition, node);
                        node.left = leftChild;
                        BuildTree(leftChild, node);
                    }

                    // Build the right child of the current node
                    Vector3 rightChildPosition = childrenPositions.Skip(1).FirstOrDefault();
                    if (rightChildPosition != Vector3.zero)
                    {
                        Node rightChild = new Node(rightChildPosition, node);
                        node.right = rightChild;
                        BuildTree(rightChild, node);
                    }

                    // Build the third child of the current node
                    Vector3 thirdChildPosition = childrenPositions.Skip(2).FirstOrDefault();
                    if (thirdChildPosition != Vector3.zero)
                    {
                        Node thirdChild = new Node(thirdChildPosition, node);
                        node.third = thirdChild;
                        BuildTree(thirdChild, node);
                    }
                    if (pre != null) childrenPositions.Add(pre.pos);
                }
            }
            public void PrintTree()
            {
                Debug.Log("NodesNum: " + nodesNum);
                PrintNode(this.root);
            }

            private void PrintNode(Node node)
            {
                if (node == null)
                {
                    return;
                }

                Debug.Log("pos: " + node.pos);
                Debug.Log("child num: " + node.childrenCount);
                PrintNode(node.left);
                PrintNode(node.right);
                PrintNode(node.third);
                Debug.DrawLine(node.pos, new Vector3(node.pos.x+1, 1f, node.pos.z+1), Color.red, 10f);
            }

            private int CountNodes(Node node)
            {
                if (node is null)
                {
                    return 0; 
                }
                else
                {
                    return 1 + CountNodes(node.left) + CountNodes(node.right) + CountNodes(node.third);
                }
            }

            public void CountSubtreeNodes(Node node)
            {
                if (node is null) { return; }
                node.childrenCount = CountNodes(node);
                CountSubtreeNodes(node.left);
                CountSubtreeNodes(node.right);
                CountSubtreeNodes(node.third);
            }

            public void visitNode(Node node, int subtreeNodesNum)
            {
                if (node is null) return;
                if (node.childrenCount >= subtreeNodesNum - 10 && node.childrenCount <= subtreeNodesNum + 10)
                {
                    if (this.root2 is null)
                    {
                        this.root2 = node;
                    }
                    else
                    {
                        this.root3 = node;
                    }
                }
                visitNode(node.left, subtreeNodesNum);
                visitNode(node.right, subtreeNodesNum);
            }
        }

        private int cur_pathway_index = 0;
        private bool is_hit = false;
        private bool stop = false;
        private float hit_time = 0;
        private bool pathChosen = false;

        private void FixedUpdate()
        {
            
            if (!pathChosen)
            {
                float dis1 = Vector3.Distance(transform.position, start_pos[0]);
                float dis2 = Vector3.Distance(transform.position, start_pos[1]);
                float dis3 = Vector3.Distance(transform.position, start_pos[2]);
                if (dis1 <= dis2 && dis1 <= dis3) pathway_points = pathway_points1;
                if (dis2 <= dis3 && dis2 <= dis1) pathway_points = pathway_points2;
                if (dis3 <= dis1 && dis3 <= dis2) pathway_points = pathway_points3;
                pathChosen = true;
                /*
                bool[] used;
                for (int i = 0; i < friends.Length; i++)
                {
                    float dis1 = Vector3.Distance(start_pos[i], pathway_points1[0]);
                    float dis2 = Vector3.Distance(start_pos[i], pathway_points2[0]);
                    float dis3 = Vector3.Distance(start_pos[i], pathway_points3[0]);
                    if (dis1 <= dis2 && dis1 <= dis3)
                    {
                        pathway_points = pathway_points1;
                    }
                    if (dis2 <= dis3 && dis2 <= dis1)
                    {
                        pathway_points = pathway_points2;
                    }
                    if (dis3 <= dis1 && dis3 <= dis2)
                    {
                        pathway_points = pathway_points3;
                    }
                }
                float dis1 = Vector3.Distance(transform.position, pathway_points1[0]);
                float dis2 = Vector3.Distance(transform.position, pathway_points2[0]);
                float dis3 = Vector3.Distance(transform.position, pathway_points3[0]);
                if (dis1 <= dis2 && dis1 <= dis3) pathway_points = pathway_points1;
                if (dis2 <= dis3 && dis2 <= dis1) pathway_points = pathway_points2;
                if (dis3 <= dis1 && dis3 <= dis2) pathway_points = pathway_points3;
                pathChosen = true;
                */
            }

            
            enemies = GameObject.FindGameObjectsWithTag("Enemy");

            
            if (enemies.Length == 0 || cur_pathway_index >= pathway_points.Count)
            {
                stop = true;
                float end_time = Time.time;
                float duration = end_time - start_time;
                Debug.Log("Time: " + duration);
            }

            if (Vector3.Distance(pathway_points[cur_pathway_index], transform.position) <= 7f)
            {
                cur_pathway_index++;
            }
            RaycastHit hit;
            if (Physics.Raycast(transform.position + transform.up, transform.TransformDirection(Vector3.forward), out hit, 3))
            {
                if ((Time.time - hit_time) > 3)
                {
                    is_hit = true;
                    hit_time = Time.time;
                }
            }
            if (stop)
            {
                m_Car.Move(0f, 0f, 0f, 1f);
            }
            else if (is_hit)
            {
                if ((Time.time - hit_time) < 2)
                {
                    m_Car.Move(0f, 0f, -0.6f, 0f);
                }
                else if ((Time.time - hit_time) < 1)
                {
                    m_Car.Move(0f, 0f, 0f, 1f);
                }
                else
                {
                    is_hit = false;
                }
            }
            else
            {
                Vector3 direction = (pathway_points[cur_pathway_index] - transform.position).normalized;

                bool is_to_the_right = Vector3.Dot(direction, transform.right) > 0f;
                bool is_to_the_front = Vector3.Dot(direction, transform.forward) > 0f;

                float steering = 0f;
                float acceleration = 0;

                float speed = 10f;
                Vector3 next_pos = pathway_points[cur_pathway_index];
                Vector3 car_pos = transform.position;
                Vector3 acceleration_vector = 1f * ((next_pos - car_pos) + m_Car_rb.velocity * (speed - m_Car_rb.velocity.magnitude));
                

                if (is_to_the_right && is_to_the_front) //right front
                {
                    steering = 1f;
                    acceleration = 1f;
                }
                else if (is_to_the_right && !is_to_the_front) //right back
                {
                    steering = -1f;
                    acceleration = -1f;
                }
                else if (!is_to_the_right && is_to_the_front) //left front
                {
                    steering = -1f;
                    acceleration = 1f;
                }
                else if (!is_to_the_right && !is_to_the_front) //left back
                {
                    steering = 1f;
                    acceleration = -1f;
                }

                if(!is_to_the_front && Vector3.Angle(next_pos - car_pos, transform.forward) < 120f)
                {
                    steering = -steering;
                }

                m_Car.Move(steering, acceleration, acceleration, 0f);
            }

            // Plot
            int i = terrain_manager.myInfo.get_i_index(transform.position.x);
            int j = terrain_manager.myInfo.get_j_index(transform.position.z);
            float grid_center_x = terrain_manager.myInfo.get_x_pos(i);
            float grid_center_z = terrain_manager.myInfo.get_z_pos(j);

            Debug.DrawLine(transform.position, new Vector3(grid_center_x, 0f, grid_center_z));

            foreach (Vector3 pos in spanningTree.Keys)
            {
                foreach (Vector3 adj in spanningTree[pos])
                {
                    Debug.DrawLine(pos, adj, Color.yellow, 2.5f);
                }
            }
            Debug.DrawLine(transform.position, pathway_points[cur_pathway_index], Color.black);

            foreach (Vector3 from in path1_edges.Keys)
            {
                Vector3 to = path1_edges[from];
                Debug.DrawLine(from, to, Color.red, 2f);
            }
            foreach (Vector3 from in path2_edges.Keys)
            {
                Vector3 to = path2_edges[from];
                Debug.DrawLine(from, to, Color.blue, 2f);
            }
            foreach (Vector3 from in path3_edges.Keys)
            {
                Vector3 to = path3_edges[from];
                Debug.DrawLine(from, to, Color.black, 2f);
            }
        }
    }
}
