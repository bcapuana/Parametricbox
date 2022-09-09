
#include <Core/CoreAll.h>
#include <Fusion/FusionAll.h>
#include <CAM/CAMAll.h>
#include <string>
using namespace adsk::core;
using namespace adsk::fusion;
using namespace adsk::cam;

Ptr<Application> app;
Ptr<UserInterface> ui;

void MakeBaseShape(Ptr<Component> component, Ptr<Sketch> sketch)
{
	Ptr<SketchCurves> curves = sketch->sketchCurves();
	Ptr<SketchLines> lines = curves->sketchLines();
	Ptr<GeometricConstraints> constraints = sketch->geometricConstraints();
	Ptr<SketchDimensions> dimensions = sketch->sketchDimensions();
	Ptr<UserParameters> parameters = ((Ptr<Design>)app->activeProduct())->userParameters();

	double length = parameters->itemByName("Length")->value();
	double width = parameters->itemByName("Width")->value();
	double height = parameters->itemByName("Height")->value();
	double thickness = parameters->itemByName("Thickness")->value();
	double drawerExtension = parameters->itemByName("DrawerExtension")->value();

	// Add main part of box
	lines->addTwoPointRectangle(Point3D::create(0, 0, 0), Point3D::create(length, width));
	lines->item(0)->deleteMe();

	int numLines = lines->count();
	Ptr<SketchLine> mainRight = lines->item(0);
	Ptr<SketchLine> mainBack = lines->item(1);
	Ptr<SketchLine> mainLeft = lines->item(2);

	// add constraints and dimensions
	constraints->addCoincident(mainLeft->endSketchPoint(), sketch->originPoint());
	constraints->addVertical(mainRight);
	constraints->addHorizontal(mainBack);
	constraints->addVertical(mainLeft);
	Ptr<SketchDimension> dim = dimensions->addDistanceDimension(mainLeft->startSketchPoint(), mainRight->endSketchPoint(), HorizontalDimensionOrientation, Point3D::create(length / 2.0, width + .5));
	dim->parameter()->expression("Length");


	// add front of box
	lines->addTwoPointRectangle(Point3D::create(-drawerExtension, -thickness), Point3D::create(width + drawerExtension, 0));
	Ptr<SketchLine> frontFront = lines->item(3);
	Ptr<SketchLine> frontRight = lines->item(4);
	Ptr<SketchLine> frontBack = lines->item(5);
	Ptr<SketchLine> frontLeft = lines->item(6);


	constraints->addHorizontal(frontFront);
	dim = dimensions->addDistanceDimension(frontBack->startSketchPoint(), mainBack->endSketchPoint(), VerticalDimensionOrientation, Point3D::create(length + .5, width / 2.0));
	dim->parameter()->expression("Width");


	constraints->addVertical(frontRight);
	constraints->addHorizontal(frontBack);
	constraints->addVertical(frontLeft);
	dim = dimensions->addDistanceDimension(frontFront->startSketchPoint(), frontBack->endSketchPoint(), VerticalDimensionOrientation, Point3D::create(length + drawerExtension + .5, .5));
	dim->parameter()->expression("Thickness");
	dim = dimensions->addDistanceDimension(mainLeft->endSketchPoint(), frontLeft->startSketchPoint(), HorizontalDimensionOrientation, Point3D::create(-(drawerExtension + 1), .5));
	dim->parameter()->expression("DrawerExtension");
	dim = dimensions->addDistanceDimension(mainRight->startSketchPoint(), frontRight->endSketchPoint(), HorizontalDimensionOrientation, Point3D::create(length + drawerExtension + 1, .5));
	dim->parameter()->expression("DrawerExtension");

	frontBack->trim(Point3D::create(width / 2.0, 0));

	numLines = lines->count();
	// add the inside profile
	lines->addTwoPointRectangle(Point3D::create(thickness, 0, 0), Point3D::create(length - thickness, length - thickness));
	numLines = lines->count();
	Ptr<SketchLine>
		insideFront = lines->item(8),
		insideRight = lines->item(9),
		insideBack = lines->item(10),
		insideLeft = lines->item(11);

	constraints->addParallel(frontFront, insideFront);
	constraints->addParallel(insideRight, mainRight);
	constraints->addParallel(insideBack, mainBack);
	constraints->addParallel(insideLeft, mainLeft);
	dim = dimensions->addDistanceDimension(insideFront->startSketchPoint(), frontFront->startSketchPoint(), VerticalDimensionOrientation, Point3D::create(-2));
	dim->parameter()->expression("Thickness");
	dim = dimensions->addDistanceDimension(insideRight->startSketchPoint(), mainRight->startSketchPoint(), HorizontalDimensionOrientation, Point3D::create(-2));
	dim->parameter()->expression("Thickness");
	dim = dimensions->addDistanceDimension(insideBack->startSketchPoint(), mainBack->startSketchPoint(), VerticalDimensionOrientation, Point3D::create(-2));
	dim->parameter()->expression("Thickness");
	dim = dimensions->addDistanceDimension(insideLeft->startSketchPoint(), mainLeft->startSketchPoint(), HorizontalDimensionOrientation, Point3D::create(-2));
	dim->parameter()->expression("Thickness");



	Ptr<ObjectCollection> oc = ObjectCollection::create();
	oc->add(sketch->profiles()->item(0));
	oc->add(sketch->profiles()->item(1));
	Ptr<ExtrudeFeatureInput> input1 = component->features()->extrudeFeatures()->createInput(oc, NewBodyFeatureOperation);

	input1->setDistanceExtent(false, ValueInput::createByString("Thickness"));
	Ptr<Feature> f1= component->features()->extrudeFeatures()->add(input1);
	f1->name("BottomExtrude");
	Ptr<Feature> baseFeature = component->features()->extrudeFeatures()->addSimple(sketch->profiles()->item(0), ValueInput::createByString("Height"), JoinFeatureOperation);
	baseFeature->name("BoxExtrude");

}
void MakeHandle(Ptr<Component> component)
{
	Ptr<Sketch> sketch = component->sketches()->add(component->xYConstructionPlane());
	Ptr<SketchCurves> curves = sketch->sketchCurves();
	Ptr<SketchLines> lines = curves->sketchLines();
	Ptr<GeometricConstraints> constraints = sketch->geometricConstraints();
	Ptr<SketchDimensions> dimensions = sketch->sketchDimensions();
	Ptr<UserParameters> parameters = ((Ptr<Design>)app->activeProduct())->userParameters();

	double length = parameters->itemByName("Length")->value();
	double thickness = parameters->itemByName("Thickness")->value();
	double handleWidth = parameters->itemByName("HandleWidth")->value();
	double handleLength = parameters->itemByName("HandleLength")->value();

	Ptr<ObjectCollection> edges = sketch->projectCutEdges(component->bRepBodies()->itemByName("MainBody"));
	for (int i = 0; i < edges->count(); i++)
	{
		Ptr<SketchLine> line = (Ptr<SketchLine>)edges->item(i);
		line->isConstruction(true);
	}





	double x1 = handleLength / 2.0 + length / 2.0;
	double x2 = -handleLength / 2.0 + length / 2.0;
	double x3 = (-handleLength * .75) / 2.0 + length / 2.0;
	double x4 = (handleLength * .75) / 2.0 + length / 2.0;


	Ptr<SketchLine> l1 = lines->addByTwoPoints(Point3D::create(x1, -thickness), Point3D::create(x2, -thickness));
	Ptr<SketchLine> l2 = lines->addByTwoPoints(Point3D::create(x2, -thickness), Point3D::create(x3, -handleWidth - thickness));

	Ptr<SketchLine> l3 = lines->addByTwoPoints(Point3D::create(x3, -handleWidth - thickness), Point3D::create(x4, -handleWidth));
	Ptr<SketchLine> l4 = lines->addByTwoPoints(Point3D::create(x4, -handleWidth - thickness), Point3D::create(x1, -thickness));

	Ptr<SketchLine> frontFront = (Ptr<SketchLine>) edges->item(3);

	constraints->addCollinear(frontFront, l1);
	constraints->addCoincident(l1->endSketchPoint(), l2->startSketchPoint());
	constraints->addCoincident(l2->endSketchPoint(), l3->startSketchPoint());
	constraints->addCoincident(l3->endSketchPoint(), l4->startSketchPoint());
	constraints->addCoincident(l4->endSketchPoint(), l1->startSketchPoint());
	constraints->addHorizontal(l1);
	constraints->addParallel(l3, l1);


	app->activeViewport()->refresh();
	app->activeViewport()->fit();

	Ptr<SketchDimension> dim = dimensions->addDistanceDimension(l1->startSketchPoint(), l1->endSketchPoint(), HorizontalDimensionOrientation, Point3D::create(length / 2.0, 1));
	dim->parameter()->expression("HandleLength");

	dim = dimensions->addDistanceDimension(l3->startSketchPoint(), l3->endSketchPoint(), HorizontalDimensionOrientation, Point3D::create(length / 2.0, -1));
	dim->parameter()->expression("HandleLength * .75");

	dim = dimensions->addDistanceDimension(l1->endSketchPoint(), l2->endSketchPoint(), VerticalDimensionOrientation, Point3D::create(length / 2.0 - handleLength / 2.0 - 1, -1));
	dim->parameter()->expression("HandleWidth");

	Ptr<SketchPoint> mp1 = sketch->sketchPoints()->add(Point3D::create((x1 + x2) / 2.0, -thickness));
	constraints->addMidPoint(mp1, l1);
	constraints->addCoincident(mp1, l1);
	dim = dimensions->addDistanceDimension(sketch->originPoint(), mp1, HorizontalDimensionOrientation, Point3D::create(length / 3.0, 1));
	dim->parameter()->expression("Length / 2.0");

	Ptr<SketchPoint> mp2 = sketch->sketchPoints()->add(Point3D::create((x1 + x2) / 2.0, -thickness));
	constraints->addMidPoint(mp2, l3);
	constraints->addCoincident(mp2, l3);
	constraints->addVerticalPoints(mp1, mp2);

	Ptr<Feature> handleExtrude = component->features()->extrudeFeatures()->addSimple(sketch->profiles()->item(0), ValueInput::createByString("Thickness"), JoinFeatureOperation);
	handleExtrude->name("HandleExtrude");

	Ptr<BRepEdges> handleEdges = handleExtrude->bodies()->item(0)->edges();
	Ptr<FilletFeatures> fillets = component->features()->filletFeatures();
	Ptr<FilletFeatureInput> filletInput = fillets->createInput();
	Ptr<ObjectCollection> filletEdges = ObjectCollection::create();

	/*for (int i = 7; i < 10; i++)
		filletEdges->add(handleEdges->item(i));*/

	filletEdges->add(handleEdges->item(1));
	filletEdges->add(handleEdges->item(3));
	filletEdges->add(handleEdges->item(6));
	filletEdges->add(handleEdges->item(8));
	filletEdges->add(handleEdges->item(10));
	filletInput->addConstantRadiusEdgeSet(filletEdges, ValueInput::createByString("2mm"), true);
	
	fillets->add(filletInput);
}
void AddChamfers(Ptr<Component> component)
{
	Ptr<ChamferFeatures> chamfers = component->features()->chamferFeatures();
	Ptr<BRepBody> body = component->bRepBodies()->item(0);
	Ptr<ObjectCollection> faces = ObjectCollection::create();
	Ptr<ObjectCollection> edges = ObjectCollection::create();
	faces->add(body->faces()->item(1));
	for (int i = 0; i < body->edges()->count() && i < 10; i++)
	{
		edges->add(body->edges()->item(i));
	}

	
	Ptr<ChamferFeatureInput> input = chamfers->createInput2();
	input->edges(edges);
	input->setToEqualDistance(ValueInput::createByString("1 mm"));
	chamfers->add(input);
}

extern "C" XI_EXPORT bool run(const char* context)
{
	app = Application::get();
	if (!app)
		return false;

	ui = app->userInterface();
	if (!ui)
		return false;

	Ptr<Design> activeProduct = app->activeProduct();
	Ptr<Component> root = activeProduct->rootComponent();
	Ptr<Occurrence> boxOccurance = root->occurrences()->addNewComponent(Matrix3D::create());



	boxOccurance->activate();
	
	Ptr<Component> boxComponent = boxOccurance->component();
	

	/*for (int i = 0; i < sketches->count(); i++)
		sketches->item(i)->deleteMe();*/

	// Setup User Parameters
	Ptr<UserParameters> parameters = activeProduct->userParameters();
	



	double length = 138;
	double width = 138;
	double height = 80;
	double drawerExtension = 5;
	double nozzleSize = 1.0;
	int numBoundaryExtrustions = 2;
	double thickness = nozzleSize * (double)numBoundaryExtrustions * 2;


	parameters->add("Length", ValueInput::createByReal(length / 10.0), "mm", "Length of Box");
	parameters->add("Width", ValueInput::createByReal(width / 10.0), "mm", "Width of Box");
	parameters->add("Height", ValueInput::createByReal(height / 10.0), "mm", "Height of Box");
	parameters->add("DrawerExtension", ValueInput::createByReal(drawerExtension / 10.0), "mm", "Front of Box Extension");
	parameters->add("NozzleSize", ValueInput::createByReal(nozzleSize / 10.0), "mm", "Size of the 3D Printer Nozzle");
	parameters->add("NumberBoundaryExtrustions", ValueInput::createByReal(numBoundaryExtrustions), "", "The number of boundary extrusions");
	Ptr<ValueInput> input = ValueInput::createByString("NozzleSize * NumberBoundaryExtrustions * 2.0");
	Ptr<Parameter> param = parameters->add("Thickness", input, "mm", "Wall Thickness");
	parameters->add("HandleLength", ValueInput::createByString("Length / 3.0"), "mm", "The length of the handle");
	parameters->add("HandleWidth", ValueInput::createByReal(1.5), "mm", "Width of the handle");



	
	Ptr<Sketch> baseSketch = boxComponent->sketches()->add(boxComponent->xYConstructionPlane());
	baseSketch->name("BaseProfile");
	MakeBaseShape(boxComponent, baseSketch);
	boxComponent->bRepBodies()->item(0)->name("MainBody");
	app->activeViewport()->refresh();
	app->activeViewport()->fit();
	MakeHandle(boxComponent);
	//AddChamfers(boxComponent);
	

	return true;
}

#ifdef XI_WIN

#include <windows.h>

BOOL APIENTRY DllMain(HMODULE hmodule, DWORD reason, LPVOID reserved)
{
	switch (reason)
	{
	case DLL_PROCESS_ATTACH:
	case DLL_THREAD_ATTACH:
	case DLL_THREAD_DETACH:
	case DLL_PROCESS_DETACH:
		break;
	}
	return TRUE;
}

#endif // XI_WIN
