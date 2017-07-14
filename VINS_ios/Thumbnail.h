//  Thumbnail.h
//  VINS_ios
//
//  Created by 李健宽 on 2017/7/13.
//  Copyright © 2017年 李健宽. All rights reserved.
//


#import <UIKit/UIKit.h>

@interface ThumbnailView : UIView
{
	int pos_x;
	int pos_y;
	
	
}

- (void) setPixelX:(int)pixel_x;
- (void) setPixelY:(int)pixel_y;


@end
