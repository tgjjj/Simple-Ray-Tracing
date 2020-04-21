#include "RayTracing.h"

int main() {
	const bool CombinationMode = true;
	
	const bool FromScratch = true; //重新渲染还是从已有的存档开始
	const string txtFileName = "37";  //已有存档的名称

	const char* fileDir = "E:/TG/Homework/RuiWang/CG2019-master/cbox/cbox.obj";
	const char* baseDir = "E:/TG/Homework/RuiWang/CG2019-master/cbox/";
	Vec cameraPos(278, 273, -800); //定义x,y,z轴为，x轴向右，y轴向上，z轴垂直屏幕向外
	Vec cameraLookAt(278, 273, -799);
	Vec cameraUp(0, 1, 0); //diningroom的cameraUp和cameraDir不正交，因此渲染出来和标准结果有所不同。
	double Fov = 39.3077;
	int resW = 512;//分辨率的宽和高
	int resH = 512;
	int samplePPix = 5000;  //SPP
	int savePSample = 5;  //每几次SPP存储一次

	if (CombinationMode) { //将三个进程跑出来的结果整合
		const string fileBaseDir = "E:/TG/Homework/RuiWang/Ray_Tracing/RT/RayTracing/Combine/";
		const string fileName1 = "80t1";
		const string fileName2 = "80t2";
		const string fileName3 = "80t3";
		vector<vector<Vec>> frameBuffer1;  //存放图像的buffer
		vector<Vec> tempVV(resH);
		frameBuffer1.resize(resW, tempVV);
		vector<vector<Vec>> frameBuffer2(frameBuffer1);
		vector<vector<Vec>> frameBuffer3(frameBuffer1);
		vector<vector<Vec>> frameBufferOutput(frameBuffer1);
		int sampleNum1, sampleNum2, sampleNum3;
		sampleNum1 = loadFromTXT(frameBuffer1, fileBaseDir + fileName1);
		sampleNum2 = loadFromTXT(frameBuffer2, fileBaseDir + fileName2);
		sampleNum3 = loadFromTXT(frameBuffer3, fileBaseDir + fileName3);
		std::cout << sampleNum1 << " " << sampleNum2 << " " << sampleNum3 << endl;
		for (int i = 0; i < resW; i++) {
			for (int j = 0; j < resH; j++) {
				//frameBuffer1[i][j].toString();
				Vec temp = (frameBuffer1[i][j] + frameBuffer2[i][j] + frameBuffer3[i][j]) * (1.0 / 3);
				//temp.toString();
				frameBufferOutput[i][j].x = temp.x;
				frameBufferOutput[i][j].y = temp.y;
				frameBufferOutput[i][j].z = temp.z;
				//frameBufferOutput[i][j].toString();
			}
			std::cout << i << endl;
		}
		int sampleNumOutput = sampleNum1 + sampleNum2 + sampleNum3;
		std::cout << sampleNumOutput << endl;
		saveToTXT(frameBufferOutput, fileBaseDir + to_string(sampleNumOutput), sampleNumOutput - 1);  //输出txt存档
		double scale = 240;
		cv::Mat output(resH, resW, CV_8UC3, cv::Scalar(0, 0, 0));
		for (int i = 0; i < resW; i++) {
			for (int j = 0; j < resH; j++) { //色调映射，先变为1/2.2次，然后再乘以固定的值
				double red = pow(frameBufferOutput[i][j].x, (1 / 2.2)) * scale;
				double green = pow(frameBufferOutput[i][j].y, (1 / 2.2)) * scale;
				double blue = pow(frameBufferOutput[i][j].z, (1 / 2.2)) * scale;
				//double red = frameBufferOutput[i][j].x * scale;
				//double green = frameBufferOutput[i][j].y * scale;
				//double blue = frameBufferOutput[i][j].z * scale;
				output.at<cv::Vec3b>(resH - j - 1, i)[0] = blue > 255 ? 255 : blue; //* scale; 这里进行了修改，OpenCV的三通道颜色是BGR所以蓝色和红色要反过来
				output.at<cv::Vec3b>(resH - j - 1, i)[1] = green > 255 ? 255 : green; //* scale;
				output.at<cv::Vec3b>(resH - j - 1, i)[2] = red > 255 ? 255 : red; //* scale;
			}
		}
		imwrite(fileBaseDir + to_string(sampleNumOutput) + ".png", output);  //输出图像
		return 0;
	}

	//int resMin = resW > resH ? resH : resW;  //小一点的那个分辨率
	Vec cameraDir = (cameraLookAt - cameraPos).norm();//相机朝向
	Vec cameraRight = (cameraDir % cameraUp).norm(); //如果cameraDir和cameraUp没有正交咋办？相机向右的方向，建立新的直角坐标系。
	cameraUp = (cameraRight % cameraDir).norm();
	double delta = tan(Fov * Pi / 360) * 2 / resW;  //这里计算了将虚拟的屏幕放在离照相机镜头1距离时，相邻像素间的距离。由于Fov的定义不同，因此渲染diningroom场景的时候需要手动改成resW
	tinyobj::attrib_t attrib;
	std::vector<tinyobj::shape_t> shapes;
	std::vector<tinyobj::material_t> materials;
	loadObj(fileDir, baseDir, true, attrib, shapes, materials); //读取obj和mtl文件

	unsigned seed = time(0);  //随机数生成器，用于随机采样和计算概率
	std::srand(seed);

	vector<vector<Vec>> frameBuffer;  //存放图像的buffer
	vector<Vec> tempVV(resH);  
	frameBuffer.resize(resW, tempVV);  //提前申请好固定大小
	int sampleNum = 0;
	if (!FromScratch) {
		sampleNum = loadFromTXT(frameBuffer, txtFileName);  //读取存档文件
	}
	vector<vector<Triangle>> faces;  //存放面片信息
	vector<vector<int>> materialIDs;  //存放每个面片对应的材质ID
	vector<BBox> bBoxes;  //存放每个面片组的bounding box，bounding box也都由三角面片构成，和建模的物体一样，这样方便后面计算
	for (auto shape : shapes) {  
		double minX = Inf;
		double minY = Inf;
		double minZ = Inf;
		double maxX = -Inf;
		double maxY = -Inf;
		double maxZ = -Inf;
		vector<Triangle> shapeFaces;
		vector<int> shapeMaterialIDs;
		for (int j = 0; j < shape.mesh.num_face_vertices.size(); j++) { //从tinyobjloader的数据结构里读取面片信息，存放到自己的数据结构里。
			int vertexIndexA = shape.mesh.indices[j * 3].vertex_index;
			int vertexIndexB = shape.mesh.indices[j * 3 + 1].vertex_index;
			int vertexIndexC = shape.mesh.indices[j * 3 + 2].vertex_index;
			Vec A(attrib.vertices[vertexIndexA * 3], attrib.vertices[vertexIndexA * 3 + 1], attrib.vertices[vertexIndexA * 3 + 2]);
			Vec B(attrib.vertices[vertexIndexB * 3], attrib.vertices[vertexIndexB * 3 + 1], attrib.vertices[vertexIndexB * 3 + 2]);
			Vec C(attrib.vertices[vertexIndexC * 3], attrib.vertices[vertexIndexC * 3 + 1], attrib.vertices[vertexIndexC * 3 + 2]);
			Triangle T(A, B, C);
			shapeFaces.push_back(T);
			shapeMaterialIDs.push_back(shape.mesh.material_ids[j]);  //这行后面一大段都是生成bounding box。
			if (A.x > maxX) { maxX = A.x; }
			if (B.x > maxX) { maxX = B.x; }
			if (C.x > maxX) { maxX = C.x; }
			if (A.y > maxY) { maxY = A.y; }
			if (B.y > maxY) { maxY = B.y; }
			if (C.y > maxY) { maxY = C.y; }
			if (A.z > maxZ) { maxZ = A.z; }
			if (B.z > maxZ) { maxZ = B.z; }
			if (C.z > maxZ) { maxZ = C.z; }
			if (A.x < minX) { minX = A.x; }
			if (B.x < minX) { minX = B.x; }
			if (C.x < minX) { minX = C.x; }
			if (A.y < minY) { minY = A.y; }
			if (B.y < minY) { minY = B.y; }
			if (C.y < minY) { minY = C.y; }
			if (A.z < minZ) { minZ = A.z; }
			if (B.z < minZ) { minZ = B.z; }
			if (C.z < minZ) { minZ = C.z; }
		}
		BBox bBox(minX, minY, minZ, maxX, maxY, maxZ);
		subBBox(bBox, shapeFaces, shapeMaterialIDs, bBoxes, faces, materialIDs);
		//faces.push_back(shapeFaces);
		//materialIDs.push_back(shapeMaterialIDs);
		//bBoxes.push_back(bBox);
		std::cout << "Total " << bBoxes.size() << " BBoxes" << endl;
	}//到这里模型读取和预处理工作基本结束，注释掉的是一些测试代码
	/*bBoxes[167].toString();
	bBoxes[188].toString();
	bBoxes[277].toString();
	return 0;*/
	/*Vec targetA(389.262, 235.746, 228.695);
	Vec targetB(400.412, 189.878, 230.809);
	Vec targetC(389.262, 189.878, 228.695);
	for (int i = 0; i < faces.size(); i++) {
		for (int j = 0; j < faces[i].size(); j++) {
			if (faces[i][j].a.equal(targetA) && faces[i][j].b.equal(targetB) && faces[i][j].b.equal(targetB)) {
				cout << "Target Found " << i << " " << j << endl;
			}
		}
	}
	return 0;*/
	/*for (int i = 0; i < faces.size(); i++) { //cbox::faceID:10和11是光源，2是光源材质
		if (materialIDs[i] == 2) {
			cout << i << endl;
		}
	}*/
	/*Vec a(0, 0, 0);
	Vec b(6, 6, 0);
	Vec c(0, 6, 0);
	Vec o(9, 3, 0);
	Vec d(-1, 0, 0);
	Ray r(o, d);
	Triangle tr(a, b, c);
	auto start = system_clock::now();
	for (int i = 0; i < 100000000; i++) {
		double t = tr.intersect(r);
	}
	auto end = system_clock::now();
	cout << duration_cast<microseconds>(end - start).count() << endl;*/
	/*Ray r(cameraPos, cameraDir);
	auto start = system_clock::now();
	Vec sampleRad = rayTrace(r, faces, materialIDs, materials);
	auto end = system_clock::now();
	cout << duration_cast<microseconds>(end - start).count() << endl;*/
	auto start = system_clock::now();
	for (int k = sampleNum; k < (samplePPix + sampleNum); k++) {//主循环，第一层是SPP,第二层是每列，第三层是每行，注释掉的是多线程的代码
		for (int i = 0; i < resW; i++) {
			for (int j = 0; j < resH; j++) {
				Vec baseRayDir = (cameraDir + cameraRight * (i - resW / 2 + 0.5)*delta + cameraUp * (j - resH / 2 + 0.5)*delta).norm(); //计算初始光线方向,位于像素中心
				double bias1r = (rand() % RandMod) / (double(RandMod) * 2); //将一个像素划分成4个super pixel，在其内部随机采样，以提高抗锯齿效果。
				double bias1u = (rand() % RandMod) / (double(RandMod) * 2);
				double bias2r = (rand() % RandMod) / (double(RandMod) * 2);
				double bias2u = (rand() % RandMod) / (double(RandMod) * 2);
				double bias3r = (rand() % RandMod) / (double(RandMod) * 2);
				double bias3u = (rand() % RandMod) / (double(RandMod) * 2);
				double bias4r = (rand() % RandMod) / (double(RandMod) * 2);
				double bias4u = (rand() % RandMod) / (double(RandMod) * 2);
				Vec rayDir1 = (baseRayDir - cameraRight * delta * bias1r - cameraUp * delta * bias1u).norm();
				//cout << "++++++++++++++++++" << endl;
				//rayDir1.toString();
				//cout << "++++++++++++++++++" << endl;
				Vec rayDir2 = (baseRayDir - cameraRight * delta * bias2r + cameraUp * delta * bias2u).norm();
				Vec rayDir3 = (baseRayDir + cameraRight * delta * bias3r - cameraUp * delta * bias3u).norm();
				Vec rayDir4 = (baseRayDir + cameraRight * delta * bias4r + cameraUp * delta * bias4u).norm();
				//baseRayDir.toString();
				//rayDir1.toString();
				//rayDir2.toString();
				//rayDir3.toString();
				//rayDir4.toString();
				//cout << "------------------------------" << endl;
				Ray r1(cameraPos, rayDir1); //生成光线
				Ray r2(cameraPos, rayDir2);
				Ray r3(cameraPos, rayDir3);
				Ray r4(cameraPos, rayDir4);
				Ray r(cameraPos, baseRayDir);
				/*Vec sampleRad1(0, 0, 0);
				Vec sampleRad2(0, 0, 0);
				Vec sampleRad3(0, 0, 0);
				Vec sampleRad4(0, 0, 0);
				thread thread1(rayTraceThr, ref(r), ref(bBoxes), ref(faces), ref(materialIDs), ref(materials), 0, ref(sampleRad1));
				thread thread2(rayTraceThr, ref(r), ref(bBoxes), ref(faces), ref(materialIDs), ref(materials), 0, ref(sampleRad2));
				thread thread3(rayTraceThr, ref(r), ref(bBoxes), ref(faces), ref(materialIDs), ref(materials), 0, ref(sampleRad3));
				thread thread4(rayTraceThr, ref(r), ref(bBoxes), ref(faces), ref(materialIDs), ref(materials), 0, ref(sampleRad4));*/
				//Vec sampleRad = rayTrace(r, bBoxes, faces, materialIDs, materials, 0, false);
				Vec sampleRad1 = rayTrace(r1, bBoxes, faces, materialIDs, materials, 0, false); //调用rayTrace函数递归地追踪光线
				Vec sampleRad2 = rayTrace(r2, bBoxes, faces, materialIDs, materials, 0, false);
				Vec sampleRad3 = rayTrace(r3, bBoxes, faces, materialIDs, materials, 0, false);
				Vec sampleRad4 = rayTrace(r4, bBoxes, faces, materialIDs, materials, 0, false);
				//sampleRad.toString();
				//sampleRad1.toString();
				//sampleRad2.toString();
				//sampleRad3.toString();
				//sampleRad4.toString();
				//sampleRad.toString();
				//thread1.join();
				//thread2.join();
				//thread3.join();
				//thread4.join();
				frameBuffer[i][j] = ((frameBuffer[i][j] * (double) k) + (sampleRad1 + sampleRad2 + sampleRad3 + sampleRad4) * (1.0 / 4)) * (1.0 / (k + 1)); //实际上就是取平均
				//frameBuffer[i][j] = ((frameBuffer[i][j] * (double)k) + sampleRad + sampleRad1 + sampleRad2 + sampleRad3 + sampleRad4) * (1.0 / (k + 5));
				//frameBuffer[i][j] = sampleRad + frameBuffer[i][j];
				//frameBuffer[i][j].toString();
				//cout << "==============================" << endl;
			}
			std::cout << k << "::" << i << endl;
		}
		//std::cout << k << endl;
		if ((k+1) % savePSample == 0) {
			auto end = system_clock::now();
			std::cout << "Time spent : " << duration_cast<seconds>(end - start).count() << endl;
			//cout << output << endl;
			double scale = 240;
			cv::Mat output(resH, resW, CV_8UC3, cv::Scalar(0, 0, 0));
			for (int i = 0; i < resW; i++) {
				for (int j = 0; j < resH; j++) { //色调映射，先变为1/2.2次，然后再乘以固定的值
					double red = pow(frameBuffer[i][j].x, (1 / 2.2)) * scale;
					double green = pow(frameBuffer[i][j].y, (1 / 2.2)) * scale;
					double blue = pow(frameBuffer[i][j].z, (1 / 2.2)) * scale;

					output.at<cv::Vec3b>(resH - j - 1, i)[0] = blue > 255 ? 255 : blue; //* scale;
					output.at<cv::Vec3b>(resH - j - 1, i)[1] = green > 255 ? 255 : green; //* scale;
					output.at<cv::Vec3b>(resH - j - 1, i)[2] = red > 255 ? 255 : red; //* scale;
				}
			}
			//imwrite("../" + to_string(k+1) + ".png", output);  //输出图像
			imwrite(to_string(k + 1) + ".png", output);  //输出图像
			saveToTXT(frameBuffer, to_string(k + 1), k);  //输出txt存档
			//imshow("image", output);
			//cv::waitKey();
		}
	}
}

void subBBox(BBox &bBox, vector<Triangle> &fatherFaces, vector<int> &fatherMaterialIDs,
	vector<BBox> &bBoxes, vector<vector<Triangle>> &faces, vector<vector<int>> &materialIDs) {
	vector<Triangle> thisBBoxFaces;
	vector<int> thisBBoxMaterialIDs;
	for (int i = 0; i < fatherFaces.size(); i ++) {
		if (bBox.intersect(fatherFaces[i])) {
			thisBBoxFaces.push_back(fatherFaces[i]);
			thisBBoxMaterialIDs.push_back(fatherMaterialIDs[i]);
		}
	}
	if (thisBBoxFaces.size() < maxBBoxFaceNum) {
		faces.push_back(thisBBoxFaces);
		materialIDs.push_back(thisBBoxMaterialIDs);
		bBoxes.push_back(bBox);
		return;
	}
	else {
		double midX = (bBox.minX + bBox.maxX) / 2;
		double midY = (bBox.minY + bBox.maxY) / 2;
		double midZ = (bBox.minZ + bBox.maxZ) / 2;
		BBox LUB(bBox.minX, bBox.minY, bBox.minZ, midX, midY, midZ);
		BBox LUF(bBox.minX, bBox.minY, midZ, midX, midY, bBox.maxZ);
		BBox LTB(bBox.minX, midY, bBox.minZ, midX, bBox.maxY, midZ);
		BBox LTF(bBox.minX, midY, midZ, midX, bBox.maxY, bBox.maxZ);
		BBox RUB(midX, bBox.minY, bBox.minZ, bBox.maxX, midY, midZ);
		BBox RUF(midX, bBox.minY, midZ, bBox.maxX, midY, bBox.maxZ);
		BBox RTB(midX, midY, bBox.minZ, bBox.maxX, bBox.maxY, midZ);
		BBox RTF(midX, midY, midZ, bBox.maxX, bBox.maxY, bBox.maxZ);
		subBBox(LUB, thisBBoxFaces, thisBBoxMaterialIDs, bBoxes, faces, materialIDs);
		subBBox(LUF, thisBBoxFaces, thisBBoxMaterialIDs, bBoxes, faces, materialIDs);
		subBBox(LTB, thisBBoxFaces, thisBBoxMaterialIDs, bBoxes, faces, materialIDs);
		subBBox(LTF, thisBBoxFaces, thisBBoxMaterialIDs, bBoxes, faces, materialIDs);
		subBBox(RUB, thisBBoxFaces, thisBBoxMaterialIDs, bBoxes, faces, materialIDs);
		subBBox(RUF, thisBBoxFaces, thisBBoxMaterialIDs, bBoxes, faces, materialIDs);
		subBBox(RTB, thisBBoxFaces, thisBBoxMaterialIDs, bBoxes, faces, materialIDs);
		subBBox(RTF, thisBBoxFaces, thisBBoxMaterialIDs, bBoxes, faces, materialIDs);
	}
	return;
}

void saveToTXT(vector<vector<Vec>> &frameBuffer, const string &fileName, int k) {  //将原始结果保存
	//ofstream out("../" + fileName + ".txt");
	ofstream out(fileName + ".txt");
	if (!out) {
		std::cout << "File open failed!" << endl;
		return;
	}
	out << k << endl;
	for (auto &col : frameBuffer) {
		for (Vec &pixel : col) {
			out << setprecision(6) << pixel.x << " " << pixel.y << " " << pixel.z << endl;
		}
	}
}

int loadFromTXT(vector<vector<Vec>> &frameBuffer, const string &fileName) {  //读取原始结果
	ifstream in(fileName + ".txt");
	if (!in) {
		std::cout << "File open failed!" << endl;
		exit(4);
	}
	int k;
	in >> k;
	for (auto &col : frameBuffer) {
		for (Vec &pixel : col) {
			in >> pixel.x >> pixel.y >> pixel.z;
		}
	}
	return k + 1;
}

inline bool nearestIntersect(const Ray &r, double &t, int &shapeID, int &faceID, 
	vector<BBox> &bBoxes, vector<vector<Triangle>> &faces) { //计算光线与面片最近的交点,距离通过t返回，面片通过两个索引shapeID和faceID返回。
	double d, inf;
	t = inf = 1e20;  //t存储到交点最短的距离
	for (int i = 0; i < faces.size(); i++) {
		if (!bBoxes[i].intersect(r)) { //判断是否与这个面片组的bounding box相交，如果不相交则跳过这个面片组
			continue;
		}
		/*if (i == 167 || i == 188 || i == 277) {
			cout << "Intersect with " << i << " Bounding Box" << endl;
		}*/
		for (int j = 0; j < faces[i].size(); j++) {  //遍历面片组中所有的面片，找到最近的交点
			d = faces[i][j].intersect(r);
			if (d != 0 && d < t) {
				shapeID = i;
				faceID = j;
				t = d;
			}
		}
	}
	return t < inf;  //如果存在交点，则返回true
}

Vec rayTrace(Ray &r, vector<BBox> &bBoxes, vector<vector<Triangle>> &faces, 
	vector<vector<int>> &materialIDs, std::vector<tinyobj::material_t> &materials, int depth, bool inObject) {//递归地追踪一条光线
	double t;
	int faceID = 0;
	int shapeID = 0;
	if (!nearestIntersect(r, t, shapeID, faceID, bBoxes, faces)) {  //找到最近的交点
		//cout << depth << endl;
		return Vec();  //不相交，返回黑色
	}
	//std::cout << "Depth: " << depth << std::endl;
	//std::cout << "T = " << t << endl;
	//faces[shapeID][faceID].toString();
	//bBoxes[shapeID].toString();
	Vec ip = r.o + r.d * t;  //计算交点坐标
	Vec ambientColor(materials[materialIDs[shapeID][faceID]].ambient[0],
		materials[materialIDs[shapeID][faceID]].ambient[1],
		materials[materialIDs[shapeID][faceID]].ambient[2]);  //发光颜色
	if (ambientColor.maxCoor() != 0) { return ambientColor; }
	Vec diffuseColor(materials[materialIDs[shapeID][faceID]].diffuse[0],
		materials[materialIDs[shapeID][faceID]].diffuse[1],
		materials[materialIDs[shapeID][faceID]].diffuse[2]);  //漫反射颜色
	//diffuseColor.toString();
	//cout << "-----------------------------------" << endl;
	double ni = materials[materialIDs[shapeID][faceID]].ior;  //折射率
	if (ni != 1.00) {  //理想折射
		Vec w = faces[shapeID][faceID].n;  //法线
		double cosTheta1 = w.dot(r.d);  //入射光线与法线夹角的余弦值
		if (cosTheta1 < 0) {  //使得法线与入射光线呈锐角
			w = w * (-1);
			cosTheta1 = (-1) * cosTheta1;
		}
		Vec u = (r.d * (1 / cosTheta1) - w).norm();  //与法线垂直，且与法线、入射光线同平面的单位向量
		double refractIndex = inObject ? ni : 1 / ni;  //光线是否在透明物体内部，由此判断是入射还是出射
		double sinTheta1 = sqrt(1 - cosTheta1 * cosTheta1); //theta1是入射角，theta2是出射角
		double sinTheta2 = sinTheta1 * refractIndex;
		Vec reflD = (r.d - w * 2 * cosTheta1).norm();
		if (sinTheta2 >= 1) {//全反射
			Ray newR(ip, reflD);
			return rayTrace(newR, bBoxes, faces, materialIDs, materials, depth, inObject);
		}
		double cosTheta2 = sqrt(1 - sinTheta2 * sinTheta2);
		double rParal = (ni * cosTheta1 - cosTheta2) / (ni * cosTheta1 + cosTheta2);
		double rVerti = (cosTheta1 - ni * cosTheta2) / (cosTheta1 + ni * cosTheta2);
		double kr = (rParal * rParal + rVerti + rVerti) / 2.0;//反射分量
		if (rand() % RandMod < kr * RandMod) {//按权重概率追踪反射光线或折射光线
			Ray newR(ip, reflD);
			return rayTrace(newR, bBoxes, faces, materialIDs, materials, depth, inObject);
		}
		else {
			Vec refrD = (w * cosTheta2 + u * sinTheta2).norm();
			Ray newR(ip, refrD);
			return rayTrace(newR, bBoxes, faces, materialIDs, materials, depth, !inObject);//反向inObject标志
		}
		//Vec newD = sinTheta2 >= 1 ? (r.d - w * 2 * cosTheta1).norm() : (w * cosTheta2 + u * sinTheta2).norm(); //sinTheta2 >= 1说明是全反射
		//inObject = sinTheta2 >= 1 ? inObject : !inObject; //全反射则还在object内部，4.2.2版本新修正
		//Ray newR(ip, newD);
		//return rayTrace(newR, bBoxes, faces, materialIDs, materials, depth, inObject);//递归追踪
	}
	if (++depth > 5) { //当追踪深度大于5时，以一定概率停止追踪
		if (depth > 20) { return ambientColor; } //出现过追踪了2000多次，栈溢出的情况，因此为了避免这种情况超过20次就直接return。
		double maxColor = diffuseColor.maxCoor();
		if ((rand() % RandMod) >= (maxColor * RandMod)) {return ambientColor;}
		else {
			//cout << maxColor << endl;
			diffuseColor = diffuseColor * (1 / maxColor);//避免因为停止追踪导致的总体亮度损失
		}
	}
	double Ns = materials[materialIDs[shapeID][faceID]].shininess;  //高光聚集度
	Vec specularColor(materials[materialIDs[shapeID][faceID]].specular[0], 
		materials[materialIDs[shapeID][faceID]].specular[1], 
		materials[materialIDs[shapeID][faceID]].specular[2]);  //高光颜色
	if (Ns <= 0) { exit(3); }
	else {  //第一种实现光泽反射和漫反射的方法，基于高光色与漫反射颜色的加权叠加，收敛较慢
		double sinThetaD;
		double sinTheta2D = rand() % RandMod / (double)RandMod;  //sin(theta)符合y=x分布，phi均匀分布
		sinThetaD = sqrt(sinTheta2D);
		Vec wD = faces[shapeID][faceID].n;  //面片法向,以此为基础建立新的直角坐标系
		if (r.d.dot(wD) > 0) { wD = wD * (-1); }
		Vec uD = ((fabs(wD.x) > 0.1 ? Vec(0, 1, 0) : Vec(1, 0, 0)) % wD).norm();
		Vec vD = wD % uD;
		double phiD = (rand() % RandMod) * 2 * Pi / double(RandMod);
		Vec newDD = (wD * sqrt(1 - sinThetaD * sinThetaD) + uD * sinThetaD * cos(phiD) + vD * sinThetaD * sin(phiD)).norm();
		Ray newRD(ip, newDD);
		Vec wS = (r.d - wD * 2 * r.d.dot(wD)).norm();
		double cosThetaS = pow(newDD.dot(wS), Ns);
		if (cosThetaS < 0) cosThetaS = 0;
		return ambientColor + (diffuseColor + (specularColor * ((Ns - 1) / 2.0) * cosThetaS)).mult(rayTrace(newRD, bBoxes, faces, materialIDs, materials, depth, inObject));
	}
	//第二种实现光泽反射和漫反射的方法，基于对反射光线方向分布的控制，收敛很快
	/*else if (Ns != 1 && ((rand() % RandMod) * 2  <= RandMod)) {//以1/2的概率追踪类镜面反射光线
		Vec wnS = faces[shapeID][faceID].n; //面片法向
		if (r.d.dot(wnS) > 0) { wnS = wnS * (-1); }
		Vec wS = (r.d - wnS * 2 * r.d.dot(wnS)).norm(); //理想镜面反射出射方向，以此为基础建立新的坐标系,w,u,v
		Vec uS = ((fabs(wS.x) > 0.1 ? Vec(0, 1, 0) : Vec(1, 0, 0)) % wS).norm();
		Vec vS = wS % uS;
		Vec newDS;
		do {
			double cosTheta2S = pow((rand() % RandMod / double(RandMod)), (1 / Ns)); //按照Ns的设置进行采样，使得(cos(theta))^(2Ns)满足分布y = 1
			double sinThetaS = sqrt(1 - cosTheta2S);
			double phiS = (rand() % RandMod) * 2 * Pi / double(RandMod);
			newDS = (wS * sqrt(1 - sinThetaS * sinThetaS) + uS * sinThetaS * cos(phiS) + vS * sinThetaS * sin(phiS)).norm();
		} while (newDS.dot(wnS) < 0);
		//newDS.toString();
		Ray newRS(ip, newDS);
		return ambientColor + specularColor.mult(rayTrace(newRS, bBoxes, faces, materialIDs, materials, depth, inObject));
		//+ diffuseColor.mult(rayTrace(newRD, bBoxes, faces, materialIDs, materials, depth, inObject)); //返回发光色加镜面反射追踪结果。
	}
	else {//追踪漫反射光线
		double sinThetaD;
		double sinTheta2D = rand() % RandMod / (double)RandMod;  //sin(theta)符合y=x分布，phi均匀分布
		sinThetaD = sqrt(sinTheta2D);
		Vec wD = faces[shapeID][faceID].n;  //面片法向,以此为基础建立新的直角坐标系
		Vec uD = ((fabs(wD.x) > 0.1 ? Vec(0, 1, 0) : Vec(1, 0, 0)) % wD).norm();
		Vec vD = wD % uD;
		double phiD = (rand() % RandMod) * 2 * Pi / double(RandMod);
		Vec newDD = (wD * sqrt(1 - sinThetaD * sinThetaD) + uD * sinThetaD * cos(phiD) + vD * sinThetaD * sin(phiD)).norm();
		Ray newRD(ip, newDD);
		return ambientColor + diffuseColor.mult(rayTrace(newRD, bBoxes, faces, materialIDs, materials, depth, inObject)); //返回发光色加上漫反射追踪返回的颜色
	}*/
}

bool loadObj(const char* fileDir, const char* baseDir, bool triangulate, tinyobj::attrib_t &attrib, 
	std::vector<tinyobj::shape_t> &shapes, std::vector<tinyobj::material_t> &materials)
{
	std::cout << "Loading " << fileDir << std::endl;
	std::string warn;
	std::string err;
	bool ret = tinyobj::LoadObj(&attrib, &shapes, &materials, &warn, &err, fileDir,
		baseDir, triangulate, true);//调用tinyobjloader的API，载入obj和mtl文件
	if (!warn.empty()) {
		std::cout << "WARN: " << warn << std::endl;
	}

	if (!err.empty()) {
		std::cerr << "ERR: " << err << std::endl;
	}

	if (!ret) {
		std::cout << "Failed to load/parse .obj.\n" << std::endl;
		return false;
	}

	//PrintInfo(attrib, shapes, materials);
	printInfoTest(attrib, shapes, materials);

	return true;
}

static void printInfoTest(const tinyobj::attrib_t& attrib,
	const std::vector<tinyobj::shape_t>& shapes,
	const std::vector<tinyobj::material_t>& materials) { //打印obj和mtl内容，调试用函数
	for (int i = 0; i < materials.size(); i ++) {
		printf("material[%ld].name = %s\n", static_cast<long>(i),
			materials[i].name.c_str());
		printf("  material.Ka = (%f, %f ,%f)\n",
			static_cast<const double>(materials[i].ambient[0]),
			static_cast<const double>(materials[i].ambient[1]),
			static_cast<const double>(materials[i].ambient[2]));
		printf("  material.Kd = (%f, %f ,%f)\n",
			static_cast<const double>(materials[i].diffuse[0]),
			static_cast<const double>(materials[i].diffuse[1]),
			static_cast<const double>(materials[i].diffuse[2]));
		printf("  material.Ks = (%f, %f ,%f)\n",
			static_cast<const double>(materials[i].specular[0]),
			static_cast<const double>(materials[i].specular[1]),
			static_cast<const double>(materials[i].specular[2]));
		printf("  material.Tr = (%f, %f ,%f)\n",
			static_cast<const double>(materials[i].transmittance[0]),
			static_cast<const double>(materials[i].transmittance[1]),
			static_cast<const double>(materials[i].transmittance[2]));
		printf("  material.Ke = (%f, %f ,%f)\n",
			static_cast<const double>(materials[i].emission[0]),
			static_cast<const double>(materials[i].emission[1]),
			static_cast<const double>(materials[i].emission[2]));
		printf("  material.Ns = %f\n",
			static_cast<const double>(materials[i].shininess));
		printf("  material.Ni = %f\n", static_cast<const double>(materials[i].ior));
		printf("  material.dissolve = %f\n",
			static_cast<const double>(materials[i].dissolve));
		printf("  material.illum = %d\n", materials[i].illum);
	}
}

static void PrintInfo(const tinyobj::attrib_t& attrib,
	const std::vector<tinyobj::shape_t>& shapes,
	const std::vector<tinyobj::material_t>& materials) { //同上一个函数
	std::cout << "# of vertices  : " << (attrib.vertices.size() / 3) << std::endl;
	std::cout << "# of normals   : " << (attrib.normals.size() / 3) << std::endl;
	std::cout << "# of texcoords : " << (attrib.texcoords.size() / 2)
		<< std::endl;

	std::cout << "# of shapes    : " << shapes.size() << std::endl;
	std::cout << "# of materials : " << materials.size() << std::endl;
	/*
	for (size_t v = 0; v < attrib.vertices.size() / 3; v++) {
		printf("  v[%ld] = (%f, %f, %f)\n", static_cast<long>(v),
			static_cast<const double>(attrib.vertices[3 * v + 0]),
			static_cast<const double>(attrib.vertices[3 * v + 1]),
			static_cast<const double>(attrib.vertices[3 * v + 2]));
	}

	for (size_t v = 0; v < attrib.normals.size() / 3; v++) {
		printf("  n[%ld] = (%f, %f, %f)\n", static_cast<long>(v),
			static_cast<const double>(attrib.normals[3 * v + 0]),
			static_cast<const double>(attrib.normals[3 * v + 1]),
			static_cast<const double>(attrib.normals[3 * v + 2]));
	}

	for (size_t v = 0; v < attrib.texcoords.size() / 2; v++) {
		printf("  uv[%ld] = (%f, %f)\n", static_cast<long>(v),
			static_cast<const double>(attrib.texcoords[2 * v + 0]),
			static_cast<const double>(attrib.texcoords[2 * v + 1]));
	}

	// For each shape
	for (size_t i = 0; i < shapes.size(); i++) {
		printf("shape[%ld].name = %s\n", static_cast<long>(i),
			shapes[i].name.c_str());
		printf("Size of shape[%ld].mesh.indices: %lu\n", static_cast<long>(i),
			static_cast<unsigned long>(shapes[i].mesh.indices.size()));
		printf("Size of shape[%ld].lines.indices: %lu\n", static_cast<long>(i),
			static_cast<unsigned long>(shapes[i].lines.indices.size()));
		printf("Size of shape[%ld].points.indices: %lu\n", static_cast<long>(i),
			static_cast<unsigned long>(shapes[i].points.indices.size()));

		size_t index_offset = 0;

		assert(shapes[i].mesh.num_face_vertices.size() ==
			shapes[i].mesh.material_ids.size());

		assert(shapes[i].mesh.num_face_vertices.size() ==
			shapes[i].mesh.smoothing_group_ids.size());

		printf("shape[%ld].num_faces: %lu\n", static_cast<long>(i),
			static_cast<unsigned long>(shapes[i].mesh.num_face_vertices.size()));

		// For each face
		for (size_t f = 0; f < shapes[i].mesh.num_face_vertices.size(); f++) {
			size_t fnum = shapes[i].mesh.num_face_vertices[f];

			printf("  face[%ld].fnum = %ld\n", static_cast<long>(f),
				static_cast<unsigned long>(fnum));

			// For each vertex in the face
			for (size_t v = 0; v < fnum; v++) {
				tinyobj::index_t idx = shapes[i].mesh.indices[index_offset + v];
				printf("    face[%ld].v[%ld].idx = %d/%d/%d\n", static_cast<long>(f),
					static_cast<long>(v), idx.vertex_index, idx.normal_index,
					idx.texcoord_index);
			}

			printf("  face[%ld].material_id = %d\n", static_cast<long>(f),
				shapes[i].mesh.material_ids[f]);
			printf("  face[%ld].smoothing_group_id = %d\n", static_cast<long>(f),
				shapes[i].mesh.smoothing_group_ids[f]);

			index_offset += fnum;
		}

		printf("shape[%ld].num_tags: %lu\n", static_cast<long>(i),
			static_cast<unsigned long>(shapes[i].mesh.tags.size()));
		for (size_t t = 0; t < shapes[i].mesh.tags.size(); t++) {
			printf("  tag[%ld] = %s ", static_cast<long>(t),
				shapes[i].mesh.tags[t].name.c_str());
			printf(" ints: [");
			for (size_t j = 0; j < shapes[i].mesh.tags[t].intValues.size(); ++j) {
				printf("%ld", static_cast<long>(shapes[i].mesh.tags[t].intValues[j]));
				if (j < (shapes[i].mesh.tags[t].intValues.size() - 1)) {
					printf(", ");
				}
			}
			printf("]");

			printf(" floats: [");
			for (size_t j = 0; j < shapes[i].mesh.tags[t].floatValues.size(); ++j) {
				printf("%f", static_cast<const double>(
					shapes[i].mesh.tags[t].floatValues[j]));
				if (j < (shapes[i].mesh.tags[t].floatValues.size() - 1)) {
					printf(", ");
				}
			}
			printf("]");

			printf(" strings: [");
			for (size_t j = 0; j < shapes[i].mesh.tags[t].stringValues.size(); ++j) {
				printf("%s", shapes[i].mesh.tags[t].stringValues[j].c_str());
				if (j < (shapes[i].mesh.tags[t].stringValues.size() - 1)) {
					printf(", ");
				}
			}
			printf("]");
			printf("\n");
		}
	}

	for (size_t i = 0; i < materials.size(); i++) {
		printf("material[%ld].name = %s\n", static_cast<long>(i),
			materials[i].name.c_str());
		printf("  material.Ka = (%f, %f ,%f)\n",
			static_cast<const double>(materials[i].ambient[0]),
			static_cast<const double>(materials[i].ambient[1]),
			static_cast<const double>(materials[i].ambient[2]));
		printf("  material.Kd = (%f, %f ,%f)\n",
			static_cast<const double>(materials[i].diffuse[0]),
			static_cast<const double>(materials[i].diffuse[1]),
			static_cast<const double>(materials[i].diffuse[2]));
		printf("  material.Ks = (%f, %f ,%f)\n",
			static_cast<const double>(materials[i].specular[0]),
			static_cast<const double>(materials[i].specular[1]),
			static_cast<const double>(materials[i].specular[2]));
		printf("  material.Tr = (%f, %f ,%f)\n",
			static_cast<const double>(materials[i].transmittance[0]),
			static_cast<const double>(materials[i].transmittance[1]),
			static_cast<const double>(materials[i].transmittance[2]));
		printf("  material.Ke = (%f, %f ,%f)\n",
			static_cast<const double>(materials[i].emission[0]),
			static_cast<const double>(materials[i].emission[1]),
			static_cast<const double>(materials[i].emission[2]));
		printf("  material.Ns = %f\n",
			static_cast<const double>(materials[i].shininess));
		printf("  material.Ni = %f\n", static_cast<const double>(materials[i].ior));
		printf("  material.dissolve = %f\n",
			static_cast<const double>(materials[i].dissolve));
		printf("  material.illum = %d\n", materials[i].illum);
		printf("  material.map_Ka = %s\n", materials[i].ambient_texname.c_str());
		printf("  material.map_Kd = %s\n", materials[i].diffuse_texname.c_str());
		printf("  material.map_Ks = %s\n", materials[i].specular_texname.c_str());
		printf("  material.map_Ns = %s\n",
			materials[i].specular_highlight_texname.c_str());
		printf("  material.map_bump = %s\n", materials[i].bump_texname.c_str());
		printf("    bump_multiplier = %f\n", static_cast<const double>(materials[i].bump_texopt.bump_multiplier));
		printf("  material.map_d = %s\n", materials[i].alpha_texname.c_str());
		printf("  material.disp = %s\n", materials[i].displacement_texname.c_str());
		printf("  <<PBR>>\n");
		printf("  material.Pr     = %f\n", static_cast<const double>(materials[i].roughness));
		printf("  material.Pm     = %f\n", static_cast<const double>(materials[i].metallic));
		printf("  material.Ps     = %f\n", static_cast<const double>(materials[i].sheen));
		printf("  material.Pc     = %f\n", static_cast<const double>(materials[i].clearcoat_thickness));
		printf("  material.Pcr    = %f\n", static_cast<const double>(materials[i].clearcoat_thickness));
		printf("  material.aniso  = %f\n", static_cast<const double>(materials[i].anisotropy));
		printf("  material.anisor = %f\n", static_cast<const double>(materials[i].anisotropy_rotation));
		printf("  material.map_Ke = %s\n", materials[i].emissive_texname.c_str());
		printf("  material.map_Pr = %s\n", materials[i].roughness_texname.c_str());
		printf("  material.map_Pm = %s\n", materials[i].metallic_texname.c_str());
		printf("  material.map_Ps = %s\n", materials[i].sheen_texname.c_str());
		printf("  material.norm   = %s\n", materials[i].normal_texname.c_str());
		std::map<std::string, std::string>::const_iterator it(
			materials[i].unknown_parameter.begin());
		std::map<std::string, std::string>::const_iterator itEnd(
			materials[i].unknown_parameter.end());

		for (; it != itEnd; it++) {
			printf("  material.%s = %s\n", it->first.c_str(), it->second.c_str());
		}
		printf("\n");
	}*/
}

