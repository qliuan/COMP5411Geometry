#include "mesh.h"
#include <iostream>
#include <igl/read_triangle_mesh.h>
#include <Eigen/Sparse>
using namespace std;


HEdge::HEdge(bool b) {
	mBoundary = b;

	mTwin = nullptr;
	mPrev = nullptr;
	mNext = nullptr;

	mStart = nullptr;
	mFace = nullptr;

	mFlag = false;
	mValid = true;
}

HEdge* HEdge::twin() const {
	return mTwin;
}

HEdge* HEdge::setTwin(HEdge* e) {
	mTwin = e;
	return mTwin;
}

HEdge* HEdge::prev() const {
	return mPrev;
}

HEdge* HEdge::setPrev(HEdge* e) {
	mPrev = e;
	return mPrev;
}

HEdge* HEdge::next() const {
	return mNext;
}

HEdge* HEdge::setNext(HEdge* e) {
	mNext = e;
	return mNext;
}

Vertex* HEdge::start() const {
	return mStart;
}

Vertex* HEdge::setStart(Vertex* v) {
	mStart = v;
	return mStart;
}

Vertex* HEdge::end() const {
	return mNext->start();
}

Face* HEdge::leftFace() const {
	return mFace;
}

Face* HEdge::setFace(Face* f) {
	mFace = f;
	return mFace;
}

bool HEdge::flag() const {
	return mFlag;
}

bool HEdge::setFlag(bool b) {
	mFlag = b;
	return mFlag;
}

bool HEdge::isBoundary() const {
	return mBoundary;
}

bool HEdge::isValid() const {
	return mValid;
}

bool HEdge::setValid(bool b) {
	mValid = b;
	return mValid;
}

OneRingHEdge::OneRingHEdge(const Vertex* v) {
	if (v == nullptr) {
		mStart = nullptr;
		mNext = nullptr;
	} else {
		mStart = v->halfEdge();
		mNext = v->halfEdge();
	}
}

HEdge* OneRingHEdge::nextHEdge() {
	HEdge* ret = mNext;
	if (mNext != nullptr && mNext->prev()->twin() != mStart) {
		mNext = mNext->prev()->twin();
	} else {
		mNext = nullptr;
	}
	return ret;
}

OneRingVertex::OneRingVertex(const Vertex* v): ring(v) {
}

Vertex* OneRingVertex::nextVertex() {
	HEdge* he = ring.nextHEdge();
	return he != nullptr ? he->end() : nullptr;
}

Vertex::Vertex() : mHEdge(nullptr), mFlag(0) {
	mPosition = Eigen::Vector3f::Zero();
	mColor = VCOLOR_BLUE;
	mNormal = Eigen::Vector3f::Zero();
}

Vertex::Vertex(const Eigen::Vector3f& v): mPosition(v), mHEdge(nullptr), mFlag(0) {
	mColor = VCOLOR_BLUE;
	mNormal = Eigen::Vector3f::Zero();
}

Vertex::Vertex(float x, float y, float z): mHEdge(nullptr), mFlag(0) {
	mPosition = Eigen::Vector3f(x, y, z);
	mColor = VCOLOR_BLUE;
	mNormal = Eigen::Vector3f::Zero();
}


const Eigen::Vector3f& Vertex::position() const {
	return mPosition;
}

const Eigen::Vector3f& Vertex::setPosition(const Eigen::Vector3f& p) {
	mPosition = p;
	return mPosition;
}

const Eigen::Vector3f& Vertex::normal() const {
	return mNormal;
}

const Eigen::Vector3f& Vertex::setNormal(const Eigen::Vector3f& n) {
	mNormal = n;
	return mNormal;
}

const Eigen::Vector3f& Vertex::color() const {
	return mColor;
}

const Eigen::Vector3f& Vertex::setColor(const Eigen::Vector3f& c) {
	mColor = c;
	return mColor;
}

HEdge* Vertex::halfEdge() const {
	return mHEdge;
}

HEdge* Vertex::setHalfEdge(HEdge* he) {
	mHEdge = he;
	return mHEdge;
}

int Vertex::index() const {
	return mIndex;
}

int Vertex::setIndex(int i) {
	mIndex = i;
	return mIndex;
}

int Vertex::flag() const {
	return mFlag;
}

int Vertex::setFlag(int f) {
	mFlag = f;
	return mFlag;
}

bool Vertex::isValid() const {
	return mValid;
}

bool Vertex::setValid(bool b) {
	mValid = b;
	return mValid;
}

bool Vertex::isBoundary() const {
	OneRingHEdge ring(this);
	HEdge* curr = nullptr;
	while (curr = ring.nextHEdge()) {
		if (curr->isBoundary()) {
			return true;
		}
	}
	return false;
}

int Vertex::valence() const {
	int count = 0;
	OneRingVertex ring(this);
	Vertex* curr = nullptr;
	while (curr = ring.nextVertex()) {
		++count;
	}
	return count;
}

Face::Face() : mHEdge(nullptr), mValid(true) {
}

HEdge* Face::halfEdge() const {
	return mHEdge;
}

HEdge* Face::setHalfEdge(HEdge* he) {
	mHEdge = he;
	return mHEdge;
}

bool Face::isBoundary() const {
	HEdge* curr = mHEdge;
	do {
		if (curr->twin()->isBoundary()) {
			return true;
		}
		curr = curr->next();
	} while (curr != mHEdge);
	return false;
}

bool Face::isValid() const {
	return mValid;
}

bool Face::setValid(bool b) {
	mValid = b;
	return mValid;
}

Mesh::Mesh() {
	mVertexPosFlag = true;
	mVertexNormalFlag = true;
	mVertexColorFlag = true;
}

Mesh::~Mesh() {
	clear();
}

const std::vector< HEdge* >& Mesh::edges() const {
	return mHEdgeList;
}

const std::vector< HEdge* >& Mesh::boundaryEdges() const {
	return mBHEdgeList;
}

const std::vector< Vertex* >& Mesh::vertices() const {
	return mVertexList;
}

const std::vector< Face* >& Mesh::faces() const {
	return mFaceList;
}


bool Mesh::isVertexPosDirty() const {
	return mVertexPosFlag;
}

void Mesh::setVertexPosDirty(bool b) {
	mVertexPosFlag = b;
}

bool Mesh::isVertexNormalDirty() const {
	return mVertexNormalFlag;
}

void Mesh::setVertexNormalDirty(bool b) {
	mVertexNormalFlag = b;
}

bool Mesh::isVertexColorDirty() const {
	return mVertexColorFlag;
}

void Mesh::setVertexColorDirty(bool b) {
	mVertexColorFlag = b;
}

bool Mesh::loadMeshFile(const std::string filename) {
	// Use libigl to parse the mesh file
	bool iglFlag = igl::read_triangle_mesh(filename, mVertexMat, mFaceMat);
	if (iglFlag) {
		clear();

		// Construct the half-edge data structure.
		int numVertices = mVertexMat.rows();
		int numFaces = mFaceMat.rows();

		// Fill in the vertex list
		for (int vidx = 0; vidx < numVertices; ++vidx) {
			mVertexList.push_back(new Vertex(mVertexMat(vidx, 0),
			                                 mVertexMat(vidx, 1),
			                                 mVertexMat(vidx, 2)));
		}
		// Fill in the face list
		for (int fidx = 0; fidx < numFaces; ++fidx) {
			addFace(mFaceMat(fidx, 0), mFaceMat(fidx, 1), mFaceMat(fidx, 2));
		}

		std::vector< HEdge* > hedgeList;

		for (int i = 0; i < mBHEdgeList.size(); ++i) {
			if (mBHEdgeList[i]->start()) {
				hedgeList.push_back(mBHEdgeList[i]);
			}
			// TODO
		}
		mBHEdgeList = hedgeList;

		for (int i = 0; i < mVertexList.size(); ++i) {
			mVertexList[i]->adjHEdges.clear();
			mVertexList[i]->setIndex(i);
			mVertexList[i]->setFlag(0);
		}
	} else {
		std::cout << __FUNCTION__ << ": mesh file loading failed!\n";
	}
	return iglFlag;
}

static void _setPrevNext(HEdge* e1, HEdge* e2) {
	e1->setNext(e2);
	e2->setPrev(e1);
}

static void _setTwin(HEdge* e1, HEdge* e2) {
	e1->setTwin(e2);
	e2->setTwin(e1);
}

static void _setFace(Face* f, HEdge* e) {
	f->setHalfEdge(e);
	e->setFace(f);
}

void Mesh::addFace(int v1, int v2, int v3) {
	Face* face = new Face();

	HEdge* hedge[3];
	HEdge* bhedge[3]; // Boundary half-edges
	Vertex* vert[3];

	for (int i = 0; i < 3; ++i) {
		hedge[i] = new HEdge();
		bhedge[i] = new HEdge(true);
	}
	vert[0] = mVertexList[v1];
	vert[1] = mVertexList[v2];
	vert[2] = mVertexList[v3];

	// Connect prev-next pointers
	for (int i = 0; i < 3; ++i) {
		_setPrevNext(hedge[i], hedge[(i + 1) % 3]);
		_setPrevNext(bhedge[i], bhedge[(i + 1) % 3]);
	}

	// Connect twin pointers
	_setTwin(hedge[0], bhedge[0]);
	_setTwin(hedge[1], bhedge[2]);
	_setTwin(hedge[2], bhedge[1]);

	// Connect start pointers for bhedge
	bhedge[0]->setStart(vert[1]);
	bhedge[1]->setStart(vert[0]);
	bhedge[2]->setStart(vert[2]);
	for (int i = 0; i < 3; ++i) {
		hedge[i]->setStart(vert[i]);
	}

	// Connect start pointers
	// Connect face-hedge pointers
	for (int i = 0; i < 3; ++i) {
		vert[i]->setHalfEdge(hedge[i]);
		vert[i]->adjHEdges.push_back(hedge[i]);
		_setFace(face, hedge[i]);
	}
	vert[0]->adjHEdges.push_back(bhedge[1]);
	vert[1]->adjHEdges.push_back(bhedge[0]);
	vert[2]->adjHEdges.push_back(bhedge[2]);

	// Merge boundary if needed
	for (int i = 0; i < 3; ++i) {
		Vertex* start = bhedge[i]->start();
		Vertex* end = bhedge[i]->end();

		for (int j = 0; j < end->adjHEdges.size(); ++j) {
			HEdge* curr = end->adjHEdges[j];
			if (curr->isBoundary() && curr->end() == start) {
				_setPrevNext(bhedge[i]->prev(), curr->next());
				_setPrevNext(curr->prev(), bhedge[i]->next());
				_setTwin(bhedge[i]->twin(), curr->twin());
				bhedge[i]->setStart(nullptr); // Mark as unused
				curr->setStart(nullptr); // Mark as unused
				break;
			}
		}
	}

	// Finally add hedges and faces to list
	for (int i = 0; i < 3; ++i) {
		mHEdgeList.push_back(hedge[i]);
		mBHEdgeList.push_back(bhedge[i]);
	}
	mFaceList.push_back(face);
}

Eigen::Vector3f Mesh::initBboxMin() const {
	return (mVertexMat.colwise().minCoeff()).transpose();
}

Eigen::Vector3f Mesh::initBboxMax() const {
	return (mVertexMat.colwise().maxCoeff()).transpose();
}

void Mesh::groupingVertexFlags() {
	// Init to 255
	for (Vertex* vert : mVertexList) {
		if (vert->flag() != 0) {
			vert->setFlag(255);
		}
	}
	// Group handles
	int id = 0;
	std::vector< Vertex* > tmpList;
	for (Vertex* vert : mVertexList) {
		if (vert->flag() == 255) {
			++id;
			vert->setFlag(id);

			// Do search
			tmpList.push_back(vert);
			while (!tmpList.empty()) {
				Vertex* v = tmpList.back();
				tmpList.pop_back();

				OneRingVertex orv = OneRingVertex(v);
				while (Vertex* v2 = orv.nextVertex()) {
					if (v2->flag() == 255) {
						v2->setFlag(id);
						tmpList.push_back(v2);
					}
				}
			}
		}
	}
}

void Mesh::clear() {
	for (int i = 0; i < mHEdgeList.size(); ++i) {
		delete mHEdgeList[i];
	}
	for (int i = 0; i < mBHEdgeList.size(); ++i) {
		delete mBHEdgeList[i];
	}
	for (int i = 0; i < mVertexList.size(); ++i) {
		delete mVertexList[i];
	}
	for (int i = 0; i < mFaceList.size(); ++i) {
		delete mFaceList[i];
	}

	mHEdgeList.clear();
	mBHEdgeList.clear();
	mVertexList.clear();
	mFaceList.clear();
}

std::vector< int > Mesh::collectMeshStats() {
	int V = 0; // # of vertices
	int E = 0; // # of half-edges
	int F = 0; // # of faces
	int B = 0; // # of boundary loops
	int C = 0; // # of connected components
	int G = 0; // # of genus

	/*====== Programming Assignment 0 ======*/

	/**********************************************/
	/*          Insert your code here.            */
	/**********************************************/

	V = mVertexList.size();
	E = mHEdgeList.size() + mBHEdgeList.size();
	F = mFaceList.size();
	B = countBoundaryLoops();
	C = countConnectedComponents();
	// v-e+f = 2(c-g)-b => g = c-(v-e+f+b)/2
	G = C-(V-E/2+F+B)/2;

	/* Collect mesh information as listed above.
	/**********************************************/

	/*====== Programming Assignment 0 ======*/

	std::vector< int > stats;
	stats.push_back(V);
	stats.push_back(E);
	stats.push_back(F);
	stats.push_back(B);
	stats.push_back(C);
	stats.push_back(G);
	return stats;
}

int Mesh::countBoundaryLoops() {
	int count = 0;

	/*====== Programming Assignment 0 ======*/

	/**********************************************/
	/*          Insert your code here.            */
	/**********************************************/

	//std::cout << "Size of mBHEdgeList " << mBHEdgeList.size() << std::endl;

	for(int i=0; i<mBHEdgeList.size(); ++i)
	{
		mBHEdgeList[i]->setFlag(false);
	}

	for(int i=0; i<mBHEdgeList.size(); ++i)
	{
		if (mBHEdgeList[i]->flag())		continue;
		count++;
		HEdge* start = mBHEdgeList[i];
		HEdge* iter = start->next();
		while(iter != start)
		{
			iter->setFlag(true);
			iter = iter->next();
		}
	}

	/*
	/* Helper function for Mesh::collectMeshStats()
	/**********************************************/

	/*====== Programming Assignment 0 ======*/

	return count;
}

int Mesh::countConnectedComponents() {
	int count = 0;

	/*====== Programming Assignment 0 ======*/

	/**********************************************/
	/*          Insert your code here.            */
	/**********************************************/

	std::vector<bool> visited (mVertexList.size(), false);
	std::vector<Vertex*> stack;

	for (int i=0; i<mVertexList.size(); i++)
	{
		if (visited[i] == true) continue;
		count++;
		stack.push_back(mVertexList[i]);

		while(stack.size() != 0)
		{
			Vertex* curr = stack.back();
			stack.pop_back();
			int j = curr->index();
			if (visited[j] == true) continue;
			visited[j] = true;

			OneRingVertex ring(curr);
			Vertex* iter = nullptr;
			while (iter = ring.nextVertex())
			{
				if (visited[iter->index()] == false) stack.push_back(iter);
			}
		}
	}


	/*
	/* Helper function for Mesh::collectMeshStats()
	/* Count the number of connected components of
	/* the mesh. (Hint: use a stack)
	/**********************************************/


	/*====== Programming Assignment 0 ======*/

	return count;
}

void Mesh::computeVertexNormals() {
	/*====== Programming Assignment 0 ======*/

	/**********************************************/
	/*          Insert your code here.            */
	/**********************************************/

	for (int i=0; i<mVertexList.size(); i++)
	{
		OneRingVertex ring(mVertexList[i]);
		Vertex* iter = nullptr;
		std::vector<Vertex*> surr;
		while(iter = ring.nextVertex())
		{
			surr.push_back(iter);
		}

		Eigen::Vector3f p2 = mVertexList[i]->position();
		std::vector<Eigen::Vector3f> norms;
		while(surr.size() > 1)
		{
			Eigen::Vector3f p1 = surr.back()->position();
			surr.pop_back();
			Eigen::Vector3f p3 = surr.back()->position();
			norms.push_back(triangleArea(p1,p2,p3)*triangleNormal(p1,p2,p3));
		}

		Eigen::Vector3f norm_sum(0,0,0);
		for(int j=0; j<norms.size(); j++)
		{
			norm_sum += norms[j];
		}

		mVertexList[i]->setNormal(norm_sum/norms.size());
	}


	/*
	/* Compute per-vertex normal using neighboring
	/* facet information. (Hint: remember using a
	/* weighting scheme. Plus, do you notice any
	/* disadvantages of your weighting scheme?)
	/**********************************************/

	/*====== Programming Assignment 0 ======*/

	// Notify mesh shaders
	setVertexNormalDirty(true);
}


void Mesh::umbrellaSmooth(bool cotangentWeights) {
	/*====== Programming Assignment 1 ======*/

	if (cotangentWeights) {
		/**********************************************/
		/*          Insert your code here.            */
		/**********************************************/

		int dim = mVertexList.size();
        Eigen::SparseMatrix<float> matrixL(dim,dim);
        Eigen::MatrixXf matrixP(dim,3);
        float lamda = 0.5;

		vector<int> indecies;
        vector<float> weights;
        float weightSum = 0;

        for (int i = 0; i < dim; ++i) {
            Vertex *ver = mVertexList[i];
            matrixP.row(i) = ver->position();
            matrixL.insert(i,i) = (float)(-1);

            // For every neighboring half edge of p0->p1
            OneRingHEdge ring(ver);
            HEdge* curr = nullptr;

            while (curr = ring.nextHEdge()) {
                // Computing weights of p1 w.r.t p0
                Eigen::Vector3f p0 = ver->position();
                Eigen::Vector3f p1 = curr->end()->position();
                Eigen::Vector3f p2 = curr->next()->end()->position();
                Eigen::Vector3f p3 = curr->twin()->prev()->start()->position();
                double cotAlpha = triangleCot(p0,p2,p1);
                double cotBeta =  triangleCot(p0,p3,p1);

                float w;
                if (!curr->end()->isBoundary()) w = ((float)(cotAlpha + cotBeta))/2;
                else w = curr->isBoundary()? (float)cotAlpha:(float)cotBeta;

                // Recording weights and indecies
                indecies.push_back(curr->end()->index());
                weights.push_back(w);
                weightSum += w;
            }

            // Set up terms related to p0 in sparse matrixL
            for(int j=0; j<indecies.size(); j++){
                matrixL.insert(i,indecies[j]) = weights[j]/weightSum;
            }

            // Rebase
            indecies.clear();
            weights.clear();
            weightSum = 0;
        }

        matrixP = matrixP + lamda*matrixL*matrixP;
        for (int i = 0; i < dim; ++i) {
            mVertexList[i]->setPosition(matrixP.row(i).transpose());
        }


		/*
		/* Step 1: Implement the cotangent weighting
		/* scheme for explicit mesh smoothing.
		/*
		/* Hint:
		/* It is advised to double type to store thee
		/* weights to avoid numerical issues.
		/**********************************************/

	} else {
		/**********************************************/
		/*          Insert your code here.            */
		/**********************************************/

		int dim = mVertexList.size();
		Eigen::MatrixXf matrixP(dim, 3);
		Eigen::SparseMatrix<float> matrixL(dim,dim);
		float lamda = 0.5;

		for (int i=0; i<dim; ++i)
		{
			Vertex *ver = mVertexList[i];
			// insert position
			matrixP.row(i) = ver->position();
			// insert laplacian
			matrixL.insert(i,i) = (float)(-1);
			int valence = ver->valence();
			OneRingVertex ring = OneRingVertex(ver);
			Vertex *iter = nullptr;
			while(iter = ring.nextVertex())
			{
				matrixL.insert(i,iter->index()) = 1/(float)valence;
			}
		}

		matrixP = matrixP + lamda*matrixL*matrixP;
		for (int i=0; i<dim; ++i)
		{
			mVertexList[i]->setPosition(matrixP.row(i).transpose());
		}

		/*
		/* Step 2: Implement the uniform weighting
		/* scheme for explicit mesh smoothing.
		/**********************************************/

	}

	/*====== Programming Assignment 1 ======*/

	computeVertexNormals();
	// Notify mesh shaders
	setVertexPosDirty(true);
}

void Mesh::implicitUmbrellaSmooth(bool cotangentWeights) {
	/*====== Programming Assignment 1 ======*/

	/* A sparse linear system Ax=b solver using the conjugate gradient method. */
	auto fnConjugateGradient = [](const Eigen::SparseMatrix< float >& A,
	                              const Eigen::VectorXf& b,
	                              int maxIterations,
	                              float errorTolerance,
	                              Eigen::VectorXf& x)
	{
		/**********************************************/
		/*          Insert your code here.            */
		/**********************************************/

		//cout << "x before computing" << endl << x << endl;

		Eigen::VectorXf r = b - A*x;
		Eigen::VectorXf p = r;
		int count = 0;
		float error = r.transpose()*r;
		while(count<maxIterations && error>errorTolerance)
		{
			float alpha = ((r.transpose()*r)/(p.transpose()*A*p))(0);
			x = x + alpha*p;
			Eigen::VectorXf rStar = r - alpha*A*p;
			float beta = ((rStar.transpose()*rStar)/(r.transpose()*r))(0);
			p = rStar + beta*p;
			r = rStar;
			error = r.transpose()*r;
			count++;
		}

		//cout << "x returning" << endl << x << endl;
		return x;

		/*
		/* Params:
		/*  A:
		/*  b:
		/*  maxIterations:	Max number of iterations
		/*  errorTolerance: Error tolerance for the early stopping condition
		/*  x:				Stores the final solution, but should be initialized.
		/**********************************************/
		/*
		/* Step 1: Implement the conjugate gradient
		/* method.
		/* Hint: https://en.wikipedia.org/wiki/Conjugate_gradient_method
		/**********************************************/
	};

	/* IMPORTANT:
	/* Please refer to the following link about the sparse matrix construction in Eigen. */
	/* http://eigen.tuxfamily.org/dox/group__TutorialSparse.html#title3 */

	int MAX_ITERATION = 1000;
	float ERROR_TOLERANCE = 1e-5;

	if (cotangentWeights) {
		/**********************************************/
		/*          Insert your code here.            */
		/**********************************************/

		int dim = mVertexList.size();
        Eigen::SparseMatrix<float> matrixA(dim,dim);
        Eigen::MatrixXf matrixP(dim,3);
        float lamda = 0.5;

		vector<int> indecies;
        vector<float> weights;
        float weightSum = 0;

        for (int i = 0; i < dim; ++i) {
            Vertex *ver = mVertexList[i];
            matrixP.row(i) = ver->position();
            matrixA.insert(i,i) = (float)1 - lamda*(float)(-1);

            // For every neighboring half edge of p0->p1
            OneRingHEdge ring(ver);
            HEdge* curr = nullptr;

            while (curr = ring.nextHEdge()) {
                // Computing weights of p1 w.r.t p0
                Eigen::Vector3f p0 = ver->position();
                Eigen::Vector3f p1 = curr->end()->position();
                Eigen::Vector3f p2 = curr->next()->end()->position();
                Eigen::Vector3f p3 = curr->twin()->prev()->start()->position();
                double cotAlpha = triangleCot(p0,p2,p1);
                double cotBeta =  triangleCot(p0,p3,p1);

                float w;
                if (!curr->end()->isBoundary()) w = ((float)(cotAlpha + cotBeta))/2;
                else w = curr->isBoundary()? (float)cotAlpha:(float)cotBeta;

                // Recording weights and indecies
                indecies.push_back(curr->end()->index());
                weights.push_back(w);
                weightSum += w;
            }

            // Set up terms related to p0 in sparse matrixA
            for(int j=0; j<indecies.size(); j++){
                matrixA.insert(i,indecies[j]) = -lamda * weights[j]/weightSum;
            }

            // Rebase
            indecies.clear();
            weights.clear();
            weightSum = 0;
        }

        // Solve A*P' = P for P'(result)
        Eigen::MatrixXf result(dim,3);
		Eigen::VectorXf col(dim);

		for (int i=0; i<dim; ++i)
		{
			col(i) = (float)0;
		}

		for (int i=0; i<3; i++)
		{
			result.col(i) = fnConjugateGradient(matrixA, matrixP.col(i), MAX_ITERATION, ERROR_TOLERANCE, col);
		}

        for (int i = 0; i < dim; ++i) {
            mVertexList[i]->setPosition(result.row(i).transpose());
        }

		/*
		/* Step 2: Implement the cotangent weighting
		/* scheme for implicit mesh smoothing. Use
		/* the above fnConjugateGradient for solving
		/* sparse linear systems.
		/*
		/* Hint:
		/* It is advised to double type to store the
		/* weights to avoid numerical issues.
		/**********************************************/


	} else {
		/**********************************************/
		/*          Insert your code here.            */
		/**********************************************/

		int dim = mVertexList.size();
		Eigen::MatrixXf matrixP(dim,3);
		Eigen::SparseMatrix<float> matrixA(dim,dim);
		float lamda = 0.5;

		for (int i=0; i<dim; ++i)
		{
			Vertex *ver = mVertexList[i];
			// Insert position
			matrixP.row(i) = ver->position();
			// Insert A = I - lamda*L
			matrixA.insert(i,i) = (float)1 - lamda*(float)(-1);
			int valence = ver->valence();
			OneRingVertex ring = OneRingVertex(ver);
			Vertex *iter = nullptr;
			while(iter = ring.nextVertex())
			{
				matrixA.insert(i,iter->index()) = -lamda/(float)valence;
			}
		}

		// Solve A*P' = P for P'(result)
		Eigen::MatrixXf result(dim,3);
		Eigen::VectorXf col(dim);

		for (int i=0; i<dim; ++i)
		{
			col(i) = (float)0;
		}

		for (int i=0; i<3; i++)
		{
			result.col(i) = fnConjugateGradient(matrixA, matrixP.col(i), MAX_ITERATION, ERROR_TOLERANCE, col);
		}

		for (int i=0; i<mVertexList.size(); ++i)
		{
			mVertexList[i]->setPosition(result.row(i).transpose());
		}

		/*
		/* Step 3: Implement the uniform weighting
		/* scheme for implicit mesh smoothing. Use
		/* the above fnConjugateGradient for solving
		/* sparse linear systems.
		/**********************************************/

	}

	/*====== Programming Assignment 1 ======*/

	computeVertexNormals();
	// Notify mesh shaders
	setVertexPosDirty(true);
}
