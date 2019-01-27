#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <string>

using namespace std;
using namespace cv;

struct node
{
	int x1,y1,x2,y2;
	float angle1,angle2;
	Vec3b pixelValue;
	node *next;
};

struct list
{
	node *head, *tail;
};

int isEmpty(list l){
	return (l.head == NULL);
}

void printList(list l){
	struct node *tmp = l.head;

	if(l.head == NULL) cout << "NULL" << endl;

	while(tmp != NULL){
		cout << tmp->x1 << " " << tmp->y1 << " " << tmp->x2 << " " << tmp->y2 << " " << tmp->pixelValue << endl;
		tmp = tmp->next;
	}
	cout << "END" << endl;
}

int len(list l){
	int i = 0;
	struct node *current;
	for(current = l.head; current != NULL; current = current -> next){
		i++;
	}
	return i;
}

void insertFirst(list &l, int x1, int y1, int x2, int y2, Vec3b pixelValue){
   struct node *link = (struct node*) malloc(sizeof(struct node));
	
   link->x1 = x1;
   link->y1 = y1;
   link->x2 = x2;
   link->y2 = y2;
   link->pixelValue = pixelValue;

   link->next = l.head;
	
   l.head = link;
   l.tail = l.head;
   l.tail->next = NULL;
}

void insertEnd(list &l, int x1, int y1, int x2, int y2, Vec3b pixelValue){
	struct node *link = (struct node*) malloc(sizeof(struct node));

	link->x1 = x1;
   	link->y1 = y1;
  	link->x2 = x2;
  	link->y2 = y2;
	link->pixelValue = pixelValue;

	l.tail->next = link;

	l.tail = link;
	l.tail->next = NULL;
}

void setred(Mat &img, int i, int j){
	img.at<Vec3b>(Point(i,j))[0] = 0;
	img.at<Vec3b>(Point(i,j))[1] = 0;
	img.at<Vec3b>(Point(i,j))[2] = 255;
}

void swap(list &l1, list &l2){
	struct node *tmp = (struct node*) malloc(sizeof(struct node));
	tmp = l1.head;
	l1.head = l2.head;
	l2.head = tmp;
}
void process(Mat &img){
	struct node *current;
	int c,d,check = 0;
	list l1,l2;
	l1.head = NULL;
	l2.head = NULL;
	int j = 0;
		for (int i=0; i<img.cols-1; i++){
			Vec3b pixel = img.at<Vec3b>(Point(i,j));
			Vec3b tmp = img.at<Vec3b>(Point(i+1,j));

			if(tmp != pixel){
				if(isEmpty(l1)){
					insertFirst(l1,0,0 ,i,j,pixel);
				}
				else insertEnd(l1,d,j,i,j,pixel);
				d = i+1;
			}
			else continue;
		}
		current = l1.head;
	// while(j<img.rows){
		j=1;
		int i = 0;
		while (i < img.cols-1){
			Vec3b pixel = img.at<Vec3b>(Point(i,j));
			Vec3b tmp = img.at<Vec3b>(Point(i+1,j));
			if(pixel != current->pixelValue){
				if(i > current->x2 && current->next != NULL){
					setred(img, i, j);
					current = current->next;
					i++;
					continue;
				}
				if(tmp == pixel){
					i++;
					continue;
				}
				c = i-1;
				if(isEmpty(l2)){
					insertFirst(l2,0,j,c,j,pixel);
				}
				else insertEnd(l2,0,j,c,j,pixel);
				// cout << i << " " << pixel << " " << current->pixelValue << endl;
				
			}
			else{
				d = i;
				if(pixel == current->pixelValue){
					i++;
					continue;
				}
				c = i-1;
				int k = 1e-3;
				if(current->angle1 == 0){
						current->angle1 = (float) 1/(d - current->x1 + k );
						current->angle2 = (float) 1/(c - current->x2 + k );
				}
				else if(1/(d - current->x1 + k) != current->angle1){
					setred(img, i, j);
				}
				else if(1/(c - current->x2 + k) != current->angle2){
					setred(img, i, j);
				}
			}
			i++;
		}
		swap(l1,l2);
		printList(l1);
		printList(l2);
		// j++;
	// }
}


int main(int argc, char const *argv[]) 
{
	Mat image;
	image = imread(argv[1], CV_LOAD_IMAGE_COLOR);

	if(!image.data){
		cout << "Could not open or find the image" << endl;
		return -1;
	}

	namedWindow("Display Window", WINDOW_AUTOSIZE);

	Vec3b pixel = image.at<Vec3b>(Point(0,0));
	// insertFirst(0,0,pixel);

	process(image);

	imshow("Display Window", image);

	waitKey(0);
	return 0;
}